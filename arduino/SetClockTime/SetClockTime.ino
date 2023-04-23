
#include "RTClib.h"

//for RTC
RTC_DS3231 rtc;

void setup() {
    if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  Serial.begin(9600);

}

void loop() {
      DateTime dateTime = rtc.now();
      char bufDate[20];
      sprintf(bufDate, "%02d-%02d-%02d %02d:%02d:%02d", dateTime.year(), dateTime.month(), dateTime.day(), dateTime.hour(), dateTime.minute(), dateTime.second()); 

      Serial.println(bufDate);
      delay(1000);
}
