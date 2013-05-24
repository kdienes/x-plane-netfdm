#define XPLM200

#include "XPLMPlugin.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include "XPLMDisplay.h"
#include "XPLMUtilities.h"
#include "XPLMDataAccess.h"
#include "XPLMProcessing.h"
#include "XPWidgets.h"
#include "XPStandardWidgets.h"

#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#include <stdio.h>
#include <assert.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <JSBSim/input_output/net_fdm.hxx>

#define PRINTERRNO() { int e = errno; printerrno (__FILE__, __LINE__, __func__, e); }

static XPLMDataRef qRef = NULL;

static XPLMDataRef localX = NULL;
static XPLMDataRef localY = NULL;
static XPLMDataRef localZ = NULL;

static XPLMDataRef planePath = NULL;

static XPLMDataRef psiRef = NULL;
static XPLMDataRef thetaRef = NULL;
static XPLMDataRef phiRef = NULL;

static XPLMDataRef latitude_ref = NULL;
static XPLMDataRef longitude_ref = NULL;
static XPLMDataRef altitude_ref = NULL;

static int s = -1;

static void printerrno (const char *file, unsigned int line, const char *func, int err)
{
  char *buf = strerror (err);
  fprintf (stderr, "error on line %d of \"%s\" (%s): %s\n", line, file, func, buf);
}

static inline float rad2degf (float r)
{
  return r * 180 / M_PI;
}

static double htond (double x)	
{
  int *p = (int *) &x;
  int tmp = p[0];
  p[0] = htonl (p[1]);
  p[1] = htonl (tmp);

  return x;
}

static float htonf (float x)	
{
  int *p = (int *) &x;
  *p = htonl (*p);
  return x;
}

static void process (FGNetFDM &fdm)
{
  fdm.version = htonl (fdm.version);

  fdm.latitude = htond (fdm.latitude);
  fdm.longitude = htond (fdm.longitude);
  fdm.altitude = htond (fdm.altitude);

  fdm.phi = htonf (fdm.phi);
  fdm.theta = htonf (fdm.theta);
  fdm.psi = htonf (fdm.psi);

  fdm.num_engines = htonl (fdm.num_engines);

  fdm.num_tanks = htonl (fdm.num_tanks);
  fdm.fuel_quantity[0] = htonf (fdm.fuel_quantity[0]);

  fdm.num_wheels = htonl (fdm.num_wheels);

  fdm.cur_time = htonl (fdm.cur_time);
  fdm.warp = htonl (fdm.warp);

  fdm.visibility = htonf (fdm.visibility);
}

static void SetOrientation (float psi, float theta, float phi)
{
#if 0
  psi /= 2; theta /= 2; phi /= 2;
  float q[4];
  
  q[0] =  cosf(psi) * cosf(theta) * cosf(phi) + sinf(psi) * sinf(theta) * sinf(phi);
  q[1] =  cosf(psi) * cosf(theta) * sinf(phi) - sinf(psi) * sinf(theta) * cosf(phi);
  q[2] =  cosf(psi) * sinf(theta) * cosf(phi) + sinf(psi) * cosf(theta) * sinf(phi);
  q[3] = -cosf(psi) * sinf(theta) * sinf(phi) + sinf(psi) * cosf(theta) * cosf(phi);

  XPLMSetDatavf (qRef, q, 0, 4);
#endif

  XPLMSetDataf (thetaRef, rad2degf (theta));
  XPLMSetDataf (phiRef, rad2degf (phi));
  XPLMSetDataf (psiRef, rad2degf (psi));

  int refs[1] = { 1 };
  XPLMSetDatavi (planePath, refs, 0, 1);
}

static bool SocketReady ()
{
  fd_set rfds;
  struct timeval tv;

  FD_ZERO (&rfds);
  FD_SET (s, &rfds);

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  int ret = select (s + 1, &rfds, NULL, NULL, &tv);
  if (ret < 0) { PRINTERRNO (); return false; }
  if (ret == 0) { return false; }
  return true;
}


static float Callback
(float sinceCall, float sinceLoop, int counter, void *ref)
{
  FGNetFDM fdm;
  struct sockaddr_in client;
  socklen_t clientlen;

  while (SocketReady ()) {

    int ret = recvfrom (s, &fdm, sizeof (fdm), 0, (struct sockaddr *) &client, &clientlen);
    if (ret != sizeof (fdm)) { PRINTERRNO (); return -1.0; }
  
    /* printf ("packet from %s:%d\n", inet_ntoa (client.sin_addr), ntohs (client.sin_port)); */
    process (fdm);
    
    double x, y, z;
    double lat, lng, alt;
    
    x = XPLMGetDatad (localX);
    y = XPLMGetDatad (localY);
    z = XPLMGetDatad (localZ);

    XPLMLocalToWorld (x, y, z, &lat, &lng, &alt);

    XPLMSetDatad (latitude_ref, lat);
    XPLMSetDatad (longitude_ref, lng);
    XPLMSetDatad (altitude_ref, alt);

    lat = fdm.latitude * 180.0 / M_PI;
    lng = fdm.longitude * 180.0 / M_PI;
    alt = fdm.altitude;

    XPLMWorldToLocal (lat, lng, alt, &x, &y, &z);

    /* printf ("lat = %f, long = %f, alt = %f\n", lat, lng, alt); */

    XPLMSetDatad (localX, x);
    XPLMSetDatad (localY, y);
    XPLMSetDatad (localZ, z);

    SetOrientation (fdm.psi, fdm.theta, fdm.phi);
  }

  return -1.0;
}

PLUGIN_API int XPluginStart (char *name, char *signature, char *description)
{
  strcpy (name, "netfdm");
  strcpy (signature, "netfdm");
  strcpy (description, "Linkage to netfdm position.");

  int xplane_ver;
  int sdk_ver;
  XPLMHostApplicationID app_id;
  XPLMGetVersions (&xplane_ver, &sdk_ver, &app_id);

  qRef = XPLMFindDataRef ("sim/flightmodel/position/q");

  latitude_ref = XPLMFindDataRef ("sim/flightmodel/position/latitude");
  longitude_ref = XPLMFindDataRef ("sim/flightmodel/position/longitude");
  altitude_ref = XPLMFindDataRef ("sim/flightmodel/position/elevation");

  localX = XPLMFindDataRef ("sim/flightmodel/position/local_x");
  localY = XPLMFindDataRef ("sim/flightmodel/position/local_y");
  localZ = XPLMFindDataRef ("sim/flightmodel/position/local_z");

  psiRef = XPLMFindDataRef ("sim/flightmodel/position/psi");
  thetaRef = XPLMFindDataRef ("sim/flightmodel/position/theta");
  phiRef = XPLMFindDataRef ("sim/flightmodel/position/phi");

  planePath = XPLMFindDataRef ("sim/operation/override/override_planepath");

  XPLMRegisterFlightLoopCallback (Callback, -1.0, NULL);
  return 1;
}

PLUGIN_API void XPluginStop (void)
{
}

PLUGIN_API void XPluginDisable (void)
{
  int ret = close (s);
  if (ret != 0) { PRINTERRNO (); return; }
  s = -1;
}

PLUGIN_API int XPluginEnable (void)
{
  struct sockaddr_in server;

  s = socket (AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s <= 0) { PRINTERRNO (); s = -1; return 0; }

  memset ((char *) &server, 0, sizeof (server));
  server.sin_family = AF_INET;
  server.sin_port = htons (5501);
  server.sin_addr.s_addr = htonl (INADDR_ANY);

  int ret = bind (s, (struct sockaddr *) &server, sizeof (server));
  if (ret != 0) { PRINTERRNO (); s = -1; return 0; }

  return 1;
}

PLUGIN_API void XPluginReceiveMessage (XPLMPluginID from, long message, void *param)
{
  if (from == XPLM_PLUGIN_XPLANE) {
    switch (message) {
    case XPLM_MSG_PLANE_LOADED:
      if (param == XPLM_PLUGIN_XPLANE) {
      }
      break;
    default:
      break;
    }
  }
}
