# serial_comm
An Example for Linux / Android Serial Communication

## How to Use in Android Source code
* Copy to the Source code Folder Just Like
```shell
android$ ls
abi     bootable  dalvik       device    frameworks  libnativehelper  out       platform_testing  serial_comm  tools
art     build     developers   docs      hardware    Makefile         packages  prebuilts         system       vendor
bionic  cts       development  external  libcore     ndk              pdk       sdk               toolchain
```

* Compile
```shell
cd serial_comm && mm -j8
```
* Test
```shell
adb remount
adb push serial_comm /system/bin/
adb shell "chmod 777 /system/bin/serial_comm"
adb shell serial_comm
```
![serial_test](https://github.com/sweetmilkcake/serial_comm/blob/master/Screenshots/serial_test.png)

## License
    Copyright 2018 SweetMilkCake

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
