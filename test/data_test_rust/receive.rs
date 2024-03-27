// use byteorder::LittleEndian;
// use byteorder::ReadBytesExt;
use std::io::Read;
use std::io::Cursor;
use std::time::Duration;

fn main(){
    let mut port = serialport::new("/dev/ttyUSB0", 115200)
        .timeout(Duration::from_millis(10))
        .open().expect("Failed to open port"); //https://crates.io/crates/serialport

    let mut serial_buf: Vec<u8> = vec![0; 12];
    port.read(serial_buf.as_mut_slice()).expect("Found no data");

    // println!("print received values");
    // println!("message in bytes: {}", serial_buf);

    let mut cursor = Cursor::new(&serial_buf);

    let mut buffer = String::new();
    cursor.read_to_string(&mut buffer).expect("Nothing to read");
    println!("Message: {}", buffer);


}