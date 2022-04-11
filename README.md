# Modbus_motor


### Compile Command 
- Compile ```motor_test_node.cpp```：```$ bash cp_file.sh```
    Run：```./motor_test_node```





### 備忘錄 Memo

目前```motor_driver```檔案寫的函數都只是```write()```的部份，未來在```main()```裡執行的時候，還需要考量```read()```的部份。也就是說也要考量```echo```或```No echo```所以會有資料長度要調整的問題。