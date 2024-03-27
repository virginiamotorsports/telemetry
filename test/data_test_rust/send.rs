// extern crate byteorder;
// extern crate serialport;
// use byteorder::LittleEndian;
// use byteorder::ReadBytesExt;
// use serialport::SerialPort;
// use std::io::Cursor;
use std::time::Duration;

fn main(){
    let mut port = serialport::new("/dev/ttyUSB1", 115200)
        .timeout(Duration::from_millis(10))
        .open().expect("Failed to open port");

    let output = "This is a test.".as_bytes();
    port.write(output).expect("Write failed");
    
}