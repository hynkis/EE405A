# Assignment for Week3
## 1. Create a ROS package
## 2. Write a ros publisher node (‘fake_sensor.py’)
* Publish a fake sensor data whose
  1. topic name is ‘/vehicle_state’
  2. message type is std_msgs/Float32 (You can set the data value arbitrary.)
  3. rate is 30Hz

## 3. Write a ros subscriber node (‘data_processor.py’)
* Subscribe to the fake sensor data
* Using the received sensor data, publish a processed data whose
  1. topic name is “/processed_state”
  2. message type is std_msgs/Float32 (You can set the processed data arbitrary.)

## 4. Run both publisher and subscriber using ‘roslaunch’
* Create a .launch script to run the publisher(‘fake_sensor.py’) and subscriber node(‘data_processor.py’).

## 5. Send followings to hynkis@kaist.ac.kr until 21.03.31 (for 2 weeks)
* Your  ROS package
* Your Report
  1. Write what you have learned this week.
  2. You can use both KOR/ENG in your report.
  3. Please submit in PDF format with the following file name, and other forms are free.
      
      EE405A_[lecture_date(YYMMDD)][Student_ID][Full name]
      
      (e.g., EE405A_210317_20215169_Hyunki_Seong.pdf)
