// yack: fixture module reaching a sibling under the root.
pub fn two() -> u32 {
    super::a::one() + 1
}
