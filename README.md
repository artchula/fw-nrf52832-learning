# fw-aider-a2
Release 5.3
Fixbug
- หน้าจอค้างหลังจากผ่านไปซักพัก 
- อุปกรณ์แสดงคำว่า Battery Full ทั้งที่ยังไม่เต็ม
- ระบบตรวจจับว่าอุปกรณ์ไม่ได้สวมใส่เพื่อเข้าสู่ Power Save Mode มีปัญหา 
- หลังจากระบบเข้าสู่โหมดประหยัดพลังงานแล้ว ระบบการอ่านแบตเตอรี่ไม่ทำงาน
- หน้าจอมีการกระพริบขณะเข้าสู่โหมดพักหน้าจอ
- หน้าจอมีการกระพริบเมื่อทำการอัพเดทเวลา
- แสดงวันผิด

Release 6
- ทำการ Print Mac Addres มาแสดง
- ปรับการเรื่องการทดสอบ Sensor ต่างๆ ให้แสดงผลดูง่ายขึ้นเวลาโปรแกรมลงตัวผลิตภัณฑ์จริง


Release 7
- ปรับการแสดงผลใหม่ โดยให้หน้าจอติดตลอดเวลา
- ปรับ Flow แก้ไขรูปแบบการอ่านปุ่มใหม่ เพื่อแก้ปัญหาหน้าจอ Slide เองตลอดเวลา
- แก้ไขตอนกด Alert ให้ไม่ต้องสั่นยาวๆ 

Release 8
-> New Feature
  -  ถ้าแขนอยู่ข้างลำตัวแล้วยกขึ้นมาดูจอจะติดเอง
 - ปรับรให้สามารถดึงข้อมูลพื้นฐานจาก Sensor ได้แก่ อัตราการเต้นหัวใจ ณ ปัจจุบัน อุณหูมิ ก้าวเดิน แคลลอรี ระยะทาง  การขอความช่วยเหลือ  แต่ยังไม่สามารถดึง Log ย้อนหลังได้
      
> Fix Bug จาก V7
  - จำนวนก้าวเดินไม่ขึ้น
 - ปรับแต่ง Setting การนับก้าวเดินให้ทำงานผิดพลาดน้อยลง 
- ปรับการแสดงผลและการกดเมนูกับมาเป็นแบบเดิม เนื่องจาก V8 กินพลังงานจนอยู่ได้เพียง 1 วัน
  
Release 9
- ปรับให้ Low Power โดยทุกอย่างต้องทำงานเหมือนเดิม แต่ต้องปรับ Flow Control ใหม่ให้ไม่มีการเรียกใช้เซนเซอร์บ่อยเกินไป
- พบเจอ Sensor ทำงานผิดปกติ คือไม่ยอมเข้าสู่โหมด Low Power ทำการแก้ไขโดยการถอดชิ้นส่วน Sensor ทังหมดออกแล้วประกอบกลับเข้าไปทีล่ะตัว
  เพื่อหาปัญญา และพบว่าเป็นที่ตัว Flash Memory ไป Config กับ BMI160 Accelerometer ทำการแก้ไขโดยการ Config pin ใหม่

Release 10
- Fix Bug Flash Memory ไม่สามาถอ่านข้อมูลได้เกิน 255 byte ต่อ Page
- Fix Bug Flash Memory ไม่สามารถเขียนข้อมูลได้ทุก Pages
- สร้างระบบเก็บข้อมูลและบันทึกลง Flash Memory
- สร้างระบบเก็บและอ่าน Configulation ลง Flash Memory
- สร้างระบบเก็บบันทึกและอ่าน Alert ต่างๆ  ลง Flash Memory
- สร้างระบบเก็บบันทึกและอ่าน  Raw data จำนวน 10S ลง  ลง Flash Memory
* เวอร์ชันนี้จะมีระบบ Log ให้แล้วแต่จะยังไม่มี Command ให้ดึงออกไปได้
* Flash Memory แบ่งเป็น 4096 Page , 1 Page มีขนาด 264 Byte
* ความยากในการพัฒนาคือข้อมูล Log แต่ล่ะอย่างมีขนาดไม่เท่ากัน
  จะต้องรู้ว่าปัจจุบัน Log แต่ล่ะอย่างเขียนถึงตำแหน่งไหน ถ้าเขียนจนเต็ม
  จะไปกลับไปเขียนจุดไหน 
* ความยากอีกเรื่องคือจะเขียน Flash บ่อยไม่ได้ดังนั้นจะต้องมีระบบ Buffer ไว้บนแรมก่อน
 พอข้อมูลจะเต็ม Buffer เมื่อไหร่ถึงจะเอาไปเขียนลง Flash
*ความยากอีกอย่างคือ ต้องเขียน Test Case เพื่อดูว่าข้อมูลที่เขียนเข้าไป กลับที่อ่านออกมาตรงกันไหน
 มีอะไรผิดพลาดบ้าง

Release 11 Alpha Test
Fixbug
- ข้อมูล Pedometer ไม่รีเซตตอนเที่ยงคืน
New Feature
- สร้างระบบ Sync กับมือถือใหม่ เพื่อให้สามารถทำงานกับ Flash Memory 
- ปรับ Algorithm คำนวณระยะทางและแคลลอรี่ใหม่
- รองรับการ Sync กับมือถือ สามารถดึงข้อมูลจากเซนเซอร์ได้ทุกอย่าง 
- รองรับการ Sync กับมือถือ เพื่อดึงค่า Log ต่างๆ เช่น การเกิด Alert เวลาไหนบ้างใน 1 ปีผ่านมา 
   หรือ  Accelerometer ตลอด 24 ชั่วโมงที่ผ่านมา เพื่อเอาไปทำ Sleep Monitoring
   หรือ HP / SPO2 ตลอด 24 ชั่วโมงที่ผ่านมาว่า เพื่อเอาไปแสดงผลภาพรวมบนมือถือ
- ปรับปรุงเรื่องประหยัดพลังงาน  คาดว่าจะสามาถใส่่ใช้งานได้ 7 วัน
- สามารถแยกแยะได้ว่าผู้สวมใส่ พักผ่อนอยู่ เดิน จ๊อกกิ่ง วิ่ง ได้
- ปรับให้ส่งข้อมูลเร็วขึ้นตอน ALert 
- เปิดการวัดหัวใจตอน Alert
*** เวอร์ชันนี้ปิดระบบ Fall Alert ออกไปก่อน เนื่องจาางกมันกินพลังงานและไม่สามารถตรวจจับการล้มได้

Release 12 Beta Test 
Fixbug
- แก้ปัญหาเรื่องอาการไม่นับก้าวเดินหลังจากผ่านเที่ยงคืน
- แก้ปัญหาอาการหน้าจอค้าง
New Feature
- ปรับการกด Alert เป็นแบบใช้มือกุมเลย

Release 13 Beta Test
- เพิ่ม WDT เพื่อป้องกันการค้าง
- แสดงผลเพียงแค่เวลาอย่างเดียว
- ปรับปรุงความแม่นยำในการแสดงผลแบตเตอรี่

Release 14 Beta Test
- ปรับการแสดงผลให้เหลือเฉพาะ HR และเวลา
- แก้ไขอาการ Hang จากการทำงานของ HR

Release 15 Beta Test
- Fix Bug เซนเซอร์ HR เสียแล้วทำให้อุปกรณ์ค้าง

Release 16 Beta Test สำหรับ Demo
- ปรับปรุงการตรวจจับ PPG ของการอ่านค่า HR ใหม่
- ใช้ Kalman Filter ในการประมาณค่าผลลัพธ์ของ HR
- ถ้าเกิดอุปกรณ์เกิดการ Reset จะทำการไปดึงค่าเวลาล่าสุดจาก Flash Memory มา
- การกดขอความช่วยเหลือต้องกดสามปุ่มพร้อมกันเท่านั้น
- ตัดการแสดงผล Temperature Sensor ออก
- จำนวนก้าวเดิน แคลลอรี่ ระยะทาง และการคาดเดากิจกรรม จะมีความแม่นยำมากขึ้นจะต้องมีการเซตค่า น้ำหนัก ส่วนสูง เพศ เท่านั้น
**** เวอร์ชันนี้ใช้สำหรับ Demo เท่านั้น

Release 17 Beta Test สำหรับ Demo
- Fix Bug ก้าวเดินไม่นับ หลังจากตื่นจาก Power Save mode
- เมื่อแบตใกล้จะมีการแจ้งเตือนที่หน้าจอ
- เมื่อแบตเหลือน้อยมาก อุปกรณ์จะเข้าสู่่โหมด Ultra Low Power Mode จะไม่สามารถใช้งานอุปกรณ์ มีเพียงการนับเวลาภายในเท่านั้นที่ทำงานอยู่

Release 18 Beta Test สำหรับทดสอบภายในบริษัท
Fixbug:
- ถ้าเกิดมีเหตุการณ์ที่ A2 เกิดตรวจจับการทำงานผิดปกติได้แล้วรีเซตขึ้นมา อุปกรณ์จะทำการดึงเวลาล่าสุดที่อยู่ใน Flash Memory
- สั่งปิด iBeacon เมื่ออุปกรณ์  Low Battery


Release 19 Beta Test สำหรับทดสอบภายในบริษัท
Fixbug:
- ใส่ WDT ใน bootloader



Release 20 Release สำหรับทดสอบภายในบริษัทFixbug:
- เปิดใช้งานทุกฟังก์ชัน


Release 21 Release สำหรับทดสอบภายในบริษัทFixbug:
- อัพเดทฟังก์ชันคำนวณ HR

Release 23 Release สำหรับทดสอบภายในบริษัทFixbug:
- อัพเดทฟังก์ชันคำนวณ SPO2 ใหม่


Release 24 Release สำหรับทดสอบภายในบริษัทFixbug:
- เพิ่มกำลังส่งสูงสุด


Release 25 Release สำหรับทดสอบภายในบริษัทFixbug:
- ปรับ Watch dog เป็น 10S

Release 26 Release สำหรับทดสอบภายในบริษัทFixbug:
- แก้ Bug SPO2 เป็น 0 บางครั้ง




