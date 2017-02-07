// Copyright (c) 2016 Rene van der Meer
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

// TODO: This module will eventually move to a separate crate for easy use in
// other projects.

extern crate libc;

use std::io;
use std::fs::OpenOptions;
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::AsRawFd;
use std::ptr;

use system::System;

#[derive (Debug)]
pub enum SlewErr {
    DevMemNotFound, DevMemPermissionDenied, DevMemIOError, DevMemMapFailed, UnknownSoC }
    
pub fn slewrate() -> Result<(),SlewErr> {
    let system = match System::new() {
        Ok(s) => s,
        Err(_) => return Err(SlewErr::UnknownSoC),
    };
    
    let mem_file = match OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(libc::O_SYNC)
        .open("/dev/mem") {
            Err(e) => {
                match e.kind() {
                    io::ErrorKind::NotFound => return Err(SlewErr::DevMemNotFound),
                    io::ErrorKind::PermissionDenied => return Err(SlewErr::DevMemPermissionDenied),
                    _ => return Err(SlewErr::DevMemIOError),
                }
            }
            Ok(file) => file,
        };
    
    let mem_ptr = unsafe {
        libc::mmap(ptr::null_mut(),
                   4096,
                   libc::PROT_READ | libc::PROT_WRITE,
                   libc::MAP_SHARED,
                   mem_file.as_raw_fd(),
                   (system.peripheral_base + 0x00100000) as libc::off_t)
//                   (0x3f100000) as libc::off_t)
    };

    if mem_ptr == libc::MAP_FAILED {
        return Err(SlewErr::DevMemMapFailed);
    }

//    println!( "hello" );
    //    println!( "{:08x}", unsafe{ *(mem_ptr.offset(0x2c) as *const u32) } );
    // 8mA drive strength, low slew rate, hysteresis on
    unsafe{ *(mem_ptr.offset(0x2c) as *mut u32) = 0x5a00000b };
//    println!( "{:08x}", unsafe{ *(mem_ptr.offset(0x2c) as *const u32) } );
    
    Ok(())
}
