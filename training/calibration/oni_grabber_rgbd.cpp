// oni_grabber_rgbd — grabber OpenNI Astra : couleur + depth aligné couleur.
//
// Identique à l'oni_grabber d'origine (Dr. José BERNARDO / ABMI), avec UN ajout :
// on écrit le champ de vision couleur (HFOV/VFOV) dans /dev/shm/oni_info.txt, ce
// qui donne les intrinsèques d'usine côté Python (fx,fy,cx,cy) sans ChArUco.
// Le depth est déjà grabé + aligné couleur (IMAGE_REGISTRATION_DEPTH_TO_COLOR)
// et écrit en /dev/shm/oni_depth.raw (16UC1, mm) — c'était déjà le cas.
//
// Build (une fois) :
//   SDK=~/Downloads/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux_x64/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/sdk
//   g++ oni_grabber_rgbd.cpp -o oni_grabber_rgbd \
//       -I$SDK/Include -L$SDK/libs -lOpenNI2 -Wl,-rpath,$SDK/libs
//   ./oni_grabber_rgbd          # laisse tourner pendant la calibration
#include <OpenNI.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

static bool write_file(const std::string& path, const void* data, size_t size){
    FILE* f = fopen(path.c_str(), "wb");
    if(!f) return false;
    size_t w = fwrite(data, 1, size, f);
    fclose(f);
    return w == size;
}

int main(){
    using namespace openni;
    if (OpenNI::initialize()!=STATUS_OK){fprintf(stderr,"OpenNI init fail: %s\n",OpenNI::getExtendedError());return 1;}
    Device dev;
    if (dev.open(ANY_DEVICE)!=STATUS_OK){fprintf(stderr,"Open device fail: %s\n",OpenNI::getExtendedError());return 2;}

    VideoStream depth, color;
    if (depth.create(dev, SENSOR_DEPTH)!=STATUS_OK){fprintf(stderr,"No depth: %s\n",OpenNI::getExtendedError());return 3;}
    if (color.create(dev, SENSOR_COLOR)!=STATUS_OK){fprintf(stderr,"No color: %s\n",OpenNI::getExtendedError());return 4;}

    // align depth to color if supported
    dev.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);

    auto setMode=[&](VideoStream& s, PixelFormat fmt){
        VideoMode m = s.getVideoMode();
        m.setResolution(640,480);
        m.setFps(30);
        m.setPixelFormat(fmt);
        s.setVideoMode(m);
    };
    setMode(color, PIXEL_FORMAT_RGB888);

    if (depth.start()!=STATUS_OK || color.start()!=STATUS_OK){
        fprintf(stderr,"Start streams failed: %s\n",OpenNI::getExtendedError());return 5;
    }

    VideoFrameRef d,c;
    bool wrote_info=false;
    while (true){
        if (depth.readFrame(&d)!=STATUS_OK || color.readFrame(&c)!=STATUS_OK) continue;
        if (!d.isValid() || !c.isValid()) continue;

        if(!wrote_info){
            FILE* f=fopen("/dev/shm/oni_info.txt","w");
            if(f){
                // dimensions (comme avant) + champ de vision couleur pour les intrinsèques
                fprintf(f,"DW=%d\nDH=%d\nCW=%d\nCH=%d\n",
                        d.getWidth(), d.getHeight(), c.getWidth(), c.getHeight());
                fprintf(f,"CHFOV=%.6f\nCVFOV=%.6f\n",
                        color.getHorizontalFieldOfView(), color.getVerticalFieldOfView());
                fclose(f);
            }
            wrote_info=true;
        }

        // dump raw depth (16UC1, mm) and color (RGB888)
        write_file("/dev/shm/oni_depth.raw", d.getData(), d.getDataSize());
        write_file("/dev/shm/oni_color.rgb", c.getData(), c.getDataSize());

        // heartbeat
        FILE* hb=fopen("/dev/shm/oni_tick.txt","w");
        if(hb){fprintf(hb,"%llu\n",(unsigned long long)d.getTimestamp()); fclose(hb);}
    }

    depth.stop(); color.stop();
    depth.destroy(); color.destroy();
    dev.close(); OpenNI::shutdown();
    return 0;
}
