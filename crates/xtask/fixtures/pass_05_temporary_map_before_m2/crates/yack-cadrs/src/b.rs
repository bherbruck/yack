// yack: fixture module reaching a sibling.
pub fn two() -> u32 {
    crate::a::one() + 1
}
