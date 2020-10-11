#[macro_use]
extern crate lazy_static;

mod black_box;

pub use black_box::BlackBox;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
