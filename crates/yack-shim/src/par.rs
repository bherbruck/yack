// yack: a rayon-shaped facade over std::thread::scope, with results in input order.
//! `par_iter()` and friends collect their items, and each `map`-like step runs on scoped threads,
//! one contiguous chunk per thread, the results concatenated in input order: what any rayon
//! program gets with pure closures, deterministically. A step started on one of these threads
//! runs on that thread alone (no nested fan-out). Searches (`any`, `all`, `find_map_any`, ...)
//! run in waves of one item per thread and answer with the first hit in input order.

use std::sync::OnceLock;

/// The prefix of the worker threads' names (how a worker knows it is one).
const WORKER: &str = "yack-par-";

/// The machine's parallelism, read once (it is a property of the machine, not state).
fn machine_threads() -> usize {
    static N: OnceLock<usize> = OnceLock::new();
    *N.get_or_init(|| std::thread::available_parallelism().map_or(1, |n| n.get()))
}

/// Whether this thread is one of the workers.
fn on_worker() -> bool {
    std::thread::current().name().is_some_and(|n| n.starts_with(WORKER))
}

/// Threads to use for `n` items here.
fn threads_for(n: usize) -> usize {
    if n < 2 || on_worker() { 1 } else { machine_threads().min(n) }
}

/// The number of threads a parallel step may use (rayon's `current_num_threads`).
pub fn current_num_threads() -> usize {
    machine_threads()
}

/// The index of this worker thread, `None` off the workers (rayon's `current_thread_index`).
pub fn current_thread_index() -> Option<usize> {
    let t = std::thread::current();
    t.name().and_then(|n| n.strip_prefix(WORKER)).and_then(|i| i.parse().ok())
}

/// `f` of each item, in order, on up to one thread per core.
fn par_map<T: Send, U: Send, F: Fn(T) -> U + Sync>(items: Vec<T>, f: &F) -> Vec<U> {
    let n = items.len();
    let threads = threads_for(n);
    if threads <= 1 {
        return items.into_iter().map(f).collect();
    }
    let chunk = n.div_ceil(threads);
    let mut chunks: Vec<Vec<T>> = Vec::with_capacity(threads);
    let mut it = items.into_iter();
    loop {
        let c: Vec<T> = it.by_ref().take(chunk).collect();
        if c.is_empty() {
            break;
        }
        chunks.push(c);
    }
    std::thread::scope(|s| {
        let mut chunks = chunks.into_iter();
        let first = chunks.next();
        let mut handles = Vec::new();
        let mut inline: Vec<(usize, Vec<T>)> = Vec::new();
        for (i, c) in chunks.enumerate() {
            let b = std::thread::Builder::new().name(format!("{WORKER}{}", i + 1));
            // A chunk whose thread couldn't start runs here instead.
            let (tx, rx) = std::sync::mpsc::channel::<Vec<T>>();
            match b.spawn_scoped(s, move || {
                let c: Vec<T> = match rx.recv() {
                    Ok(c) => c,
                    Err(_) => Vec::new(),
                };
                c.into_iter().map(f).collect::<Vec<U>>()
            }) {
                Ok(h) => {
                    let _ = tx.send(c);
                    handles.push((i, Some(h)));
                }
                Err(_) => {
                    inline.push((i, c));
                    handles.push((i, None));
                }
            }
        }
        let mut out: Vec<U> = Vec::with_capacity(n);
        if let Some(c) = first {
            out.extend(c.into_iter().map(f));
        }
        let mut inline = inline.into_iter();
        for (_, h) in handles {
            match h {
                Some(h) => match h.join() {
                    Ok(v) => out.extend(v),
                    Err(e) => std::panic::resume_unwind(e),
                },
                None => {
                    if let Some((_, c)) = inline.next() {
                        out.extend(c.into_iter().map(f));
                    }
                }
            }
        }
        out
    })
}

/// `(a(), b())`, `b` on a thread of its own when that is worth it (rayon's `join`).
pub fn join<A, B, RA, RB>(a: A, b: B) -> (RA, RB)
where
    A: FnOnce() -> RA + Send,
    B: FnOnce() -> RB + Send,
    RA: Send,
    RB: Send,
{
    if threads_for(2) < 2 {
        return (a(), b());
    }
    std::thread::scope(|s| {
        match std::thread::Builder::new().name(format!("{WORKER}1")).spawn_scoped(s, b) {
            Ok(h) => {
                let ra = a();
                match h.join() {
                    Ok(rb) => (ra, rb),
                    Err(e) => std::panic::resume_unwind(e),
                }
            }
            Err(e) => panic!("join: could not start a thread: {e}"),
        }
    })
}

/// A parallel sequence: its items, evaluated eagerly.
#[derive(Clone, Debug)]
pub struct Par<T> {
    items: Vec<T>,
}

impl<T> IntoIterator for Par<T> {
    type Item = T;
    type IntoIter = std::vec::IntoIter<T>;
    fn into_iter(self) -> Self::IntoIter {
        self.items.into_iter()
    }
}

/// rayon's `IntoParallelIterator`.
pub trait IntoParallelIterator {
    type Item: Send;
    type Iter;
    /// The items, to run steps over in parallel.
    fn into_par_iter(self) -> Par<Self::Item>;
}

impl<I: IntoIterator> IntoParallelIterator for I
where
    I::Item: Send,
{
    type Item = I::Item;
    type Iter = Par<I::Item>;
    fn into_par_iter(self) -> Par<I::Item> {
        Par { items: self.into_iter().collect() }
    }
}

/// rayon's `IntoParallelRefIterator`: `par_iter()` on anything `&x` iterates.
pub trait IntoParallelRefIterator<'a> {
    type Item: Send + 'a;
    /// The items by reference.
    fn par_iter(&'a self) -> Par<Self::Item>;
}

impl<'a, I: ?Sized + 'a> IntoParallelRefIterator<'a> for I
where
    &'a I: IntoIterator,
    <&'a I as IntoIterator>::Item: Send + 'a,
{
    type Item = <&'a I as IntoIterator>::Item;
    fn par_iter(&'a self) -> Par<Self::Item> {
        Par { items: self.into_iter().collect() }
    }
}

/// rayon's `IntoParallelRefMutIterator`: `par_iter_mut()`.
pub trait IntoParallelRefMutIterator<'a> {
    type Item: Send + 'a;
    /// The items by mutable reference.
    fn par_iter_mut(&'a mut self) -> Par<Self::Item>;
}

impl<'a, I: ?Sized + 'a> IntoParallelRefMutIterator<'a> for I
where
    &'a mut I: IntoIterator,
    <&'a mut I as IntoIterator>::Item: Send + 'a,
{
    type Item = <&'a mut I as IntoIterator>::Item;
    fn par_iter_mut(&'a mut self) -> Par<Self::Item> {
        Par { items: self.into_iter().collect() }
    }
}

/// rayon's `ParallelSlice`.
pub trait ParallelSlice<T: Sync> {
    /// Consecutive chunks of `n` (the last may be shorter).
    fn par_chunks(&self, n: usize) -> Par<&[T]>;
    /// Overlapping windows of `n`.
    fn par_windows(&self, n: usize) -> Par<&[T]>;
    /// Exact chunks of `n` (the remainder dropped).
    fn par_chunks_exact(&self, n: usize) -> Par<&[T]>;
}

impl<T: Sync> ParallelSlice<T> for [T] {
    fn par_chunks(&self, n: usize) -> Par<&[T]> {
        Par { items: self.chunks(n).collect() }
    }
    fn par_windows(&self, n: usize) -> Par<&[T]> {
        Par { items: self.windows(n).collect() }
    }
    fn par_chunks_exact(&self, n: usize) -> Par<&[T]> {
        Par { items: self.chunks_exact(n).collect() }
    }
}

/// rayon's `ParallelSliceMut`.
pub trait ParallelSliceMut<T: Send> {
    /// Consecutive mutable chunks of `n`.
    fn par_chunks_mut(&mut self, n: usize) -> Par<&mut [T]>;
    /// Sorted (stable).
    fn par_sort_by<F: Fn(&T, &T) -> std::cmp::Ordering + Sync>(&mut self, f: F);
    /// Sorted (unstable).
    fn par_sort_unstable_by<F: Fn(&T, &T) -> std::cmp::Ordering + Sync>(&mut self, f: F);
}

impl<T: Send> ParallelSliceMut<T> for [T] {
    fn par_chunks_mut(&mut self, n: usize) -> Par<&mut [T]> {
        Par { items: self.chunks_mut(n).collect() }
    }
    fn par_sort_by<F: Fn(&T, &T) -> std::cmp::Ordering + Sync>(&mut self, f: F) {
        self.sort_by(f)
    }
    fn par_sort_unstable_by<F: Fn(&T, &T) -> std::cmp::Ordering + Sync>(&mut self, f: F) {
        self.sort_unstable_by(f)
    }
}

/// rayon's `ParallelIterator` (implemented by [`Par`]; the steps are its inherent methods).
pub trait ParallelIterator {}
impl<T> ParallelIterator for Par<T> {}
/// rayon's `IndexedParallelIterator`.
pub trait IndexedParallelIterator {}
impl<T> IndexedParallelIterator for Par<T> {}

/// The first `Some` of `f` over `items` in input order, one wave of items per round.
fn first_some<T: Send, U: Send, F: Fn(T) -> Option<U> + Sync>(items: Vec<T>, f: &F) -> Option<U> {
    let wave = threads_for(items.len()).max(1);
    let mut it = items.into_iter();
    loop {
        let batch: Vec<T> = it.by_ref().take(wave).collect();
        if batch.is_empty() {
            return None;
        }
        if let Some(hit) = par_map(batch, f).into_iter().flatten().next() {
            return Some(hit);
        }
    }
}

impl<T: Send> Par<T> {
    /// The items.
    pub fn into_vec(self) -> Vec<T> {
        self.items
    }
    /// `f` of each item.
    pub fn map<U: Send, F: Fn(T) -> U + Sync + Send>(self, f: F) -> Par<U> {
        Par { items: par_map(self.items, &f) }
    }
    /// `f` of each item, with a per-thread value (here made once per item group).
    pub fn map_init<S, U: Send, I: Fn() -> S + Sync + Send, F: Fn(&mut S, T) -> U + Sync + Send>(
        self,
        init: I,
        f: F,
    ) -> Par<U> {
        let g = |x: T| {
            let mut s = init();
            f(&mut s, x)
        };
        Par { items: par_map(self.items, &g) }
    }
    /// `f` of each item with a clone of `init`.
    pub fn map_with<S: Clone + Send + Sync, U: Send, F: Fn(&mut S, T) -> U + Sync + Send>(
        self,
        init: S,
        f: F,
    ) -> Par<U> {
        let g = |x: T| {
            let mut s = init.clone();
            f(&mut s, x)
        };
        Par { items: par_map(self.items, &g) }
    }
    /// The `Some`s of `f`.
    pub fn filter_map<U: Send, F: Fn(T) -> Option<U> + Sync + Send>(self, f: F) -> Par<U> {
        Par { items: par_map(self.items, &f).into_iter().flatten().collect() }
    }
    /// The items `f` keeps.
    pub fn filter<F: Fn(&T) -> bool + Sync + Send>(self, f: F) -> Par<T> {
        let g = |x: T| if f(&x) { Some(x) } else { None };
        Par { items: par_map(self.items, &g).into_iter().flatten().collect() }
    }
    /// The items of each `f`, in order.
    pub fn flat_map<P: IntoParallelIterator, F: Fn(T) -> P + Sync + Send>(self, f: F) -> Par<P::Item>
    where
        P: Send,
    {
        let parts = par_map(self.items, &f);
        Par { items: parts.into_iter().flat_map(|p| p.into_par_iter().items).collect() }
    }
    /// The items of each `f` (a serial iterator), in order.
    pub fn flat_map_iter<I: IntoIterator, F: Fn(T) -> I + Sync + Send>(self, f: F) -> Par<I::Item>
    where
        I::Item: Send,
    {
        let g = |x: T| f(x).into_iter().collect::<Vec<I::Item>>();
        Par { items: par_map(self.items, &g).into_iter().flatten().collect() }
    }
    /// Each item's index with it.
    pub fn enumerate(self) -> Par<(usize, T)> {
        Par { items: self.items.into_iter().enumerate().collect() }
    }
    /// Paired with `other`'s items (as long as the shorter).
    pub fn zip<P: IntoParallelIterator>(self, other: P) -> Par<(T, P::Item)> {
        Par { items: self.items.into_iter().zip(other.into_par_iter().items).collect() }
    }
    /// Followed by `other`'s items.
    pub fn chain<P: IntoParallelIterator<Item = T>>(self, other: P) -> Par<T> {
        let mut items = self.items;
        items.extend(other.into_par_iter().items);
        Par { items }
    }
    /// Reversed.
    pub fn rev(self) -> Par<T> {
        let mut items = self.items;
        items.reverse();
        Par { items }
    }
    /// The first `n`.
    pub fn take(self, n: usize) -> Par<T> {
        Par { items: self.items.into_iter().take(n).collect() }
    }
    /// All but the first `n`.
    pub fn skip(self, n: usize) -> Par<T> {
        Par { items: self.items.into_iter().skip(n).collect() }
    }
    /// Every `n`th.
    pub fn step_by(self, n: usize) -> Par<T> {
        Par { items: self.items.into_iter().step_by(n).collect() }
    }
    /// Itself (a scheduling hint in rayon).
    pub fn with_min_len(self, _n: usize) -> Par<T> {
        self
    }
    /// Itself (a scheduling hint in rayon).
    pub fn with_max_len(self, _n: usize) -> Par<T> {
        self
    }
    /// Itself (rayon's panic fusing).
    pub fn panic_fuse(self) -> Par<T> {
        self
    }
    /// `f` sees each item.
    pub fn inspect<F: Fn(&T) + Sync + Send>(self, f: F) -> Par<T> {
        let g = |x: T| {
            f(&x);
            x
        };
        Par { items: par_map(self.items, &g) }
    }
    /// The items collected.
    pub fn collect<C: FromIterator<T>>(self) -> C {
        self.items.into_iter().collect()
    }
    /// Appended to `v`.
    pub fn collect_into_vec(self, v: &mut Vec<T>) {
        v.clear();
        v.extend(self.items);
    }
    /// `f` of each item, for its effects.
    pub fn for_each<F: Fn(T) + Sync + Send>(self, f: F) {
        let _ = par_map(self.items, &f);
    }
    /// `f` of each item, stopping at the first error (in input order).
    pub fn try_for_each<E: Send, F: Fn(T) -> Result<(), E> + Sync + Send>(self, f: F) -> Result<(), E> {
        par_map(self.items, &f).into_iter().collect()
    }
    /// The sum.
    pub fn sum<S: std::iter::Sum<T>>(self) -> S {
        self.items.into_iter().sum()
    }
    /// The product.
    pub fn product<S: std::iter::Product<T>>(self) -> S {
        self.items.into_iter().product()
    }
    /// The number of items.
    pub fn count(self) -> usize {
        self.items.len()
    }
    /// The number of items.
    pub fn len(&self) -> usize {
        self.items.len()
    }
    /// Whether there are none.
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }
    /// The smallest (the first of equals).
    pub fn min(self) -> Option<T>
    where
        T: Ord,
    {
        self.items.into_iter().min()
    }
    /// The largest (the last of equals).
    pub fn max(self) -> Option<T>
    where
        T: Ord,
    {
        self.items.into_iter().max()
    }
    /// The smallest by `f`.
    pub fn min_by<F: Fn(&T, &T) -> std::cmp::Ordering + Sync + Send>(self, f: F) -> Option<T> {
        self.items.into_iter().min_by(f)
    }
    /// The largest by `f`.
    pub fn max_by<F: Fn(&T, &T) -> std::cmp::Ordering + Sync + Send>(self, f: F) -> Option<T> {
        self.items.into_iter().max_by(f)
    }
    /// The smallest by key.
    pub fn min_by_key<K: Ord, F: Fn(&T) -> K + Sync + Send>(self, f: F) -> Option<T> {
        self.items.into_iter().min_by_key(f)
    }
    /// The largest by key.
    pub fn max_by_key<K: Ord, F: Fn(&T) -> K + Sync + Send>(self, f: F) -> Option<T> {
        self.items.into_iter().max_by_key(f)
    }
    /// The items folded with `op` from `identity()`.
    pub fn reduce<I: Fn() -> T + Sync + Send, F: Fn(T, T) -> T + Sync + Send>(self, identity: I, op: F) -> T {
        self.items.into_iter().fold(identity(), op)
    }
    /// The items folded with `op`, `None` when there are none.
    pub fn reduce_with<F: Fn(T, T) -> T + Sync + Send>(self, op: F) -> Option<T> {
        self.items.into_iter().reduce(op)
    }
    /// rayon's `fold`: one accumulator (rayon makes one per piece of work; a following
    /// `reduce`/`sum` gives the same answer).
    pub fn fold<A: Send, I: Fn() -> A + Sync + Send, F: Fn(A, T) -> A + Sync + Send>(self, identity: I, op: F) -> Par<A> {
        Par { items: vec![self.items.into_iter().fold(identity(), op)] }
    }
    /// Whether `f` holds for some item.
    pub fn any<F: Fn(T) -> bool + Sync + Send>(self, f: F) -> bool {
        let g = |x: T| f(x).then_some(());
        first_some(self.items, &g).is_some()
    }
    /// Whether `f` holds for every item.
    pub fn all<F: Fn(T) -> bool + Sync + Send>(self, f: F) -> bool {
        let g = |x: T| (!f(x)).then_some(());
        first_some(self.items, &g).is_none()
    }
    /// The first `Some` of `f`, in input order.
    pub fn find_map_any<U: Send, F: Fn(T) -> Option<U> + Sync + Send>(self, f: F) -> Option<U> {
        first_some(self.items, &f)
    }
    /// The first `Some` of `f`, in input order.
    pub fn find_map_first<U: Send, F: Fn(T) -> Option<U> + Sync + Send>(self, f: F) -> Option<U> {
        first_some(self.items, &f)
    }
    /// The first item `f` holds for, in input order.
    pub fn find_any<F: Fn(&T) -> bool + Sync + Send>(self, f: F) -> Option<T> {
        let g = |x: T| if f(&x) { Some(x) } else { None };
        first_some(self.items, &g)
    }
    /// The first item `f` holds for.
    pub fn find_first<F: Fn(&T) -> bool + Sync + Send>(self, f: F) -> Option<T> {
        self.find_any(f)
    }
    /// The index of the first item `f` holds for.
    pub fn position_any<F: Fn(T) -> bool + Sync + Send>(self, f: F) -> Option<usize> {
        let g = |(i, x): (usize, T)| f(x).then_some(i);
        first_some(self.items.into_iter().enumerate().collect(), &g)
    }
    /// The index of the first item `f` holds for.
    pub fn position_first<F: Fn(T) -> bool + Sync + Send>(self, f: F) -> Option<usize> {
        self.position_any(f)
    }
    /// The items split by `f` into (true, false).
    pub fn partition<A: Default + Extend<T>, B: Default + Extend<T>, F: Fn(&T) -> bool + Sync + Send>(
        self,
        f: F,
    ) -> (A, B) {
        let mut a = A::default();
        let mut b = B::default();
        for x in self.items {
            if f(&x) {
                a.extend(Some(x));
            } else {
                b.extend(Some(x));
            }
        }
        (a, b)
    }
    /// The last item.
    pub fn last(self) -> Option<T> {
        self.items.into_iter().last()
    }
}

impl<T: Send + Clone> Par<&T> {
    /// The items cloned.
    pub fn cloned(self) -> Par<T>
    where
        T: Sync,
    {
        Par { items: self.items.into_iter().cloned().collect() }
    }
}

impl<T: Send + Copy + Sync> Par<&T> {
    /// The items copied.
    pub fn copied(self) -> Par<T> {
        Par { items: self.items.into_iter().copied().collect() }
    }
}

impl<A: Send, B: Send> Par<(A, B)> {
    /// The pairs split apart.
    pub fn unzip<CA: Default + Extend<A>, CB: Default + Extend<B>>(self) -> (CA, CB) {
        self.items.into_iter().unzip()
    }
}

impl<P: IntoParallelIterator + Send> Par<P> {
    /// The items of each item.
    pub fn flatten(self) -> Par<P::Item> {
        Par { items: self.items.into_iter().flat_map(|p| p.into_par_iter().items).collect() }
    }
}

impl<I: IntoIterator + Send> Par<I>
where
    I::Item: Send,
{
    /// The items of each item (serial iterators).
    pub fn flatten_iter(self) -> Par<I::Item> {
        Par { items: self.items.into_iter().flatten().collect() }
    }
}

/// rayon's `ThreadBuilder`: what a pool's spawn handler is handed. Here no pool is made, so
/// none is ever handed out; the type exists for code written against rayon's API.
pub struct ThreadBuilder {
    name: Option<String>,
    stack_size: Option<usize>,
}

impl ThreadBuilder {
    /// The thread's name.
    pub fn name(&self) -> Option<&str> {
        self.name.as_deref()
    }
    /// The thread's stack size.
    pub fn stack_size(&self) -> Option<usize> {
        self.stack_size
    }
    /// Runs the thread's work (none here).
    pub fn run(self) {}
}

/// rayon's `ThreadPoolBuildError`.
#[derive(Debug)]
pub struct ThreadPoolBuildError;

impl std::fmt::Display for ThreadPoolBuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "the global thread pool has already been initialized")
    }
}
impl std::error::Error for ThreadPoolBuildError {}

/// rayon's `ThreadPoolBuilder`. There is no pool: steps start scoped threads as they need them,
/// so building one does nothing (and never calls the spawn handler).
#[derive(Default)]
pub struct ThreadPoolBuilder {
    _threads: usize,
}

impl ThreadPoolBuilder {
    /// A builder.
    pub fn new() -> Self {
        Self::default()
    }
    /// The thread count (ignored).
    pub fn num_threads(self, n: usize) -> Self {
        Self { _threads: n }
    }
    /// The spawn handler (never called: there is no pool).
    pub fn spawn_handler<F: FnMut(ThreadBuilder) -> std::io::Result<()>>(self, _f: F) -> Self {
        self
    }
    /// The stack size (ignored).
    pub fn stack_size(self, _n: usize) -> Self {
        self
    }
    /// "Builds" the global pool: there is none to build.
    pub fn build_global(self) -> Result<(), ThreadPoolBuildError> {
        Ok(())
    }
    /// A pool: a name for running work as usual (steps start their own threads).
    pub fn build(self) -> Result<ThreadPool, ThreadPoolBuildError> {
        Ok(ThreadPool { threads: self._threads })
    }
}

/// rayon's `ThreadPool`. Work run in it runs as anywhere else (results never depend on the
/// thread count here).
#[derive(Debug)]
pub struct ThreadPool {
    threads: usize,
}

impl ThreadPool {
    /// `f()`, here.
    pub fn install<R, F: FnOnce() -> R>(&self, f: F) -> R {
        f()
    }
    /// The thread count asked for (the machine's when none was).
    pub fn current_num_threads(&self) -> usize {
        if self.threads == 0 { machine_threads() } else { self.threads }
    }
}

/// rayon's prelude.
pub mod prelude {
    pub use super::{
        IndexedParallelIterator, IntoParallelIterator, IntoParallelRefIterator,
        IntoParallelRefMutIterator, ParallelIterator, ParallelSlice, ParallelSliceMut,
    };
}
