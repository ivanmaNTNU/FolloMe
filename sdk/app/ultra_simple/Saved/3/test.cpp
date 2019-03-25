#include "lidar.h"

using namespace rp::standalone::rplidar;





int main(){
    RPlidarDriver* drv = startLidarExpress(); // CAN ALSO USE startLidar(); instead of Express. Fewer points on normal. 
    float dist[8192];
    float angl[8192];
    int quality[8192];
    delay(1000);
    int count = lidarData(dist, angl, quality, drv);
    stopLidar(drv);
    for (int pos = 0; pos < count ; ++pos) {
            printf("theta: %03.2f Dist: %08.2f Q: %d\n",  
                angl[pos],
                dist[pos], quality[pos]);
                
    }
    return 0;
}

