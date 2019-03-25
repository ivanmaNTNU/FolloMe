/*************************************************************

  Download latest Blynk library here:

    https://github.com/blynkkk/blynk-library/releases/latest



  Blynk is a platform with iOS and Android apps to control

  Arduino, Raspberry Pi and the likes over the Internet.

  You can easily build graphic interfaces for all your

  projects by simply dragging and dropping widgets.



    Downloads, docs, tutorials: http://www.blynk.cc

    Sketch generator:           http://examples.blynk.cc

    Blynk community:            http://community.blynk.cc

    Follow us:                  http://www.fb.com/blynkapp

                                http://twitter.com/blynk_app



  Blynk library is licensed under MIT license

  This example code is in public domain.



 *************************************************************



  App project setup:

    GPS Stream widget on V1.

 *************************************************************/



/* Comment this out to disable prints and save space */

//#define BLYNK_PRINT stdout




#include <BlynkApiLinux.h>


#include <BlynkSocket.h>

#include <BlynkOptionsParser.h>
#include <iostream>
#include <iomanip>


static BlynkTransportSocket _blynkTransport;

BlynkSocket Blynk(_blynkTransport);

#include <BlynkWidgets.h>



BLYNK_WRITE(V1) {

  GpsParam gps(param);

  std::cout << std::fixed;
  std::cout << std::setprecision(7);

  // Print 6 decimal places for Lat, Lon

  std::cout << "Lat: ";

  std::cout << gps.getLat() << std::endl;



  std::cout << "Lon: ";

  std::cout << gps.getLon() << std::endl;


  // Print 2 decimal places for Alt, Speed

  std::cout << "Altitute: ";

  std::cout << gps.getAltitude() << std::endl;



  std::cout << "Speed: ";

  std::cout << gps.getSpeed() << std::endl;



   std::cout << std::endl;

}






void loop()

{

  Blynk.run();

}



void getGpsBlynk(){

  /*const char *auth, *serv;
  

  uint16_t port;
  std::cout << "argc: " << argc << std::endl;
  std::cout << "argv: " << argv[0] << std::endl;
  std::cout << "argv: " << argv[1][2] << std::endl;
  std::cout << "argv: " << *argv[1] << std::endl;

  parse_options(argc, argv, auth, serv, port);

  std::cout << "auth: " << auth << std::endl;
  std::cout << "serv: " << serv << std::endl;
  std::cout << "port: " << port << std::endl;*/
  
  //const char auth[40] = "d2c74c8ae8704342a3163d493fd8a224";
  const char auth[40] = "c91cd62590e74daf803ff0339b256f13";
  const char serv[17] = "blynk-cloud.com";
  uint16_t port = 80;

  Blynk.begin(auth, serv, port);


  while (1){
    loop();
  }
  

}




}
// --token=d2c74c8ae8704342a3163d493fd8a224

