use futures_lite::future::block_on;
use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};

fn main() {
    let di = nusb::list_devices()
        .unwrap()
        .find(|d| d.vendor_id() == 0xc0de && d.product_id() == 0xcafe)
        .expect("no device found");
    let device = di.open().expect("error opening device");
    let interface = device.claim_interface(0).expect("error claiming interface");

    // Send "hello world" to device
    let result = block_on(interface.control_out(ControlOut {
        control_type: ControlType::Vendor,
        recipient: Recipient::Interface,
        request: 100,
        value: 200,
        index: 0,
        data: b"hello world",
    }));
    println!("{result:?}");

    // Receive "hello" from device
    let result = block_on(interface.control_in(ControlIn {
        control_type: ControlType::Vendor,
        recipient: Recipient::Interface,
        request: 101,
        value: 201,
        index: 0,
        length: 5,
    }));
    println!("{result:?}");
}

