use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

pub trait Floats:
    Sub<Output = Self>
    + SubAssign
    + Add<Output = Self>
    + AddAssign
    + Neg<Output = Self>
    + Mul<Output = Self>
    + MulAssign
    + Div<Output = Self>
    + DivAssign
    + Copy
    + Sized
    + PartialOrd
    + Default
{
}

//impl Floats for f16 {}
impl Floats for f32 {}
impl Floats for f64 {}
//impl Floats for f128 {}

#[macro_export]
macro_rules! assert_flexible {
    ($value1:expr, $value2:expr, $percent:expr) => {{
        // Convert percentage to a ratio
        let ratio = $percent / 100.0;
        let result = ($value1 >= $value2 * (1.0 - ratio) && $value1 <= $value2 * (1.0 + ratio));

        assert!(result, "Assertion failed: {} !~= {}", $value1, $value2);
    }};
}