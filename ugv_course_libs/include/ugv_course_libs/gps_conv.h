#ifndef GPS_CONV_H_
#define GPS_CONV_H_

#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>

#define SOUTHERN_HEMISPHERE -1
#define UNDEFINED_HEMISPHERE 0
#define NORTHERN_HEMISPHERE 1

// WGS84 Parameters
const double WGS84_A = 6378137.0;   // major axis
const double WGS84_B = 6356752.31424518;  // minor axis
const double WGS84_F = 0.0033528107;    // ellipsoid flattening
const double WGS84_E = 0.0818191908;    // first eccentricity
const double WGS84_EP = 0.0820944379;   // second eccentricity

// UTM Parameters
const double UTM_K0 = 0.9996;     // scale factor
const double UTM_FE = 500000.0;   // false easting
const double UTM_FN_N = 0.0;      // false northing on north hemisphere
const double UTM_FN_S = 10000000.0;   // false northing on south hemisphere
const double UTM_E2 = (WGS84_E * WGS84_E);  // e^2
const double UTM_E4 = (UTM_E2 * UTM_E2);    // e^4
const double UTM_E6 = (UTM_E4 * UTM_E2);    // e^6
const double UTM_EP2 = (UTM_E2 / (1 - UTM_E2)); // e'^2

inline double deg2Rad(double deg)
{
  return deg * M_PI / 180.0;
}

inline double rad2Deg(double rad)
{
  return rad * 180.0 / M_PI;
}

static inline void LLtoECEF(const double lat, const double lon, const double alt, double& ecef_x, double& ecef_y, double& ecef_z)
{
  // Distance from ECEF z axis to the surface
  double N = WGS84_A / sqrt(1 - WGS84_E * WGS84_E * sin(lat) * sin(lat));

  // Compute equivalent ECEF coordinates
  ecef_x = (N + alt) * cos(lat) * cos(lon);
  ecef_y = (N + alt) * cos(lat) * sin(lon);
  ecef_z = (N * (1 - WGS84_E * WGS84_E) + alt) * sin(lat);
}

inline double RN(double phi)
{
  return WGS84_A / sqrt(1 - WGS84_E * WGS84_E * sin(phi) * sin(phi));
}

inline void ECEFtoLL(const double ecef_x, const double ecef_y, const double ecef_z, double& lat, double& lon, double& alt)
{
  /* Longitude is easy */
  lon = rad2Deg(atan2(ecef_y, ecef_x));

  /* Iterative solution for latitude */
  double r;
  double p = sqrt(ecef_x * ecef_x + ecef_y * ecef_y);
  lat = atan(p / ecef_z);

  for (int i = 0; i < 4; i++) {
    r = RN(lat);
    alt = p / cos(lat) - r;
    lat = atan(ecef_z / p / (1 - WGS84_E * WGS84_E * r / (r + alt)));
  }

  alt = p / cos(lat) - RN(lat);
  lat = rad2Deg(lat);
}

/**
 * Convert lat/long to UTM coords.  Equations from USGS Bulletin 1532
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees
 *
 * UTM coordinate conversion is only accurate between 80 degrees south
 * and 84 degrees north.  Details can be found at:
 * http://www.ngs.noaa.gov/TOOLS/utm.shtml
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
inline void LLtoUTM(const double Lat, const double Long, double &UTMEasting, double &UTMNorthing, int& UTMZone,
                           int& hemi)
{
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double k0 = UTM_K0;

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

  //Make sure the longitude is between -180.00 .. 179.9
  double LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180;

  double LatRad = deg2Rad(Lat);
  double LongRad = deg2Rad(LongTemp);
  double LongOriginRad;

  UTMZone = int((LongTemp + 180) / 6) + 1;

  if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
    UTMZone = 32;

  // Special zones for Svalbard
  if (Lat >= 72.0 && Lat < 84.0) {
    if (LongTemp >= 0.0 && LongTemp < 9.0)
      UTMZone = 31;
    else if (LongTemp >= 9.0 && LongTemp < 21.0)
      UTMZone = 33;
    else if (LongTemp >= 21.0 && LongTemp < 33.0)
      UTMZone = 35;
    else if (LongTemp >= 33.0 && LongTemp < 42.0)
      UTMZone = 37;
  }
  // +3 puts origin in middle of zone
  LongOrigin = (UTMZone - 1) * 6 - 180 + 3;
  LongOriginRad = deg2Rad(LongOrigin);

  //compute the UTM Zone from the latitude and longitude
  eccPrimeSquared = (eccSquared) / (1 - eccSquared);

  N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
  T = tan(LatRad) * tan(LatRad);
  C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
  A = cos(LatRad) * (LongRad - LongOriginRad);

  M = a
      * ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 - 5 * eccSquared * eccSquared * eccSquared / 256)
          * LatRad
          - (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 + 45 * eccSquared * eccSquared * eccSquared / 1024)
              * sin(2 * LatRad)
          + (15 * eccSquared * eccSquared / 256 + 45 * eccSquared * eccSquared * eccSquared / 1024) * sin(4 * LatRad)
          - (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad));

  UTMEasting = (double)(k0 * N
      * (A + (1 - T + C) * A * A * A / 6
          + (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) * A * A * A * A * A / 120) + 500000.0);

  UTMNorthing = (double)(k0
      * (M
          + N * tan(LatRad)
              * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
                  + (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) * A * A * A * A * A * A / 720)));
  if (Lat < 0) {
    UTMNorthing += 10000000.0; //10000000 meter offset for southern hemisphere
    hemi = SOUTHERN_HEMISPHERE;
  } else {
    hemi = NORTHERN_HEMISPHERE;
  }
}

/**
 * Converts UTM coords to lat/long.  Equations from USGS Bulletin 1532
 *
 * East Longitudes are positive, West longitudes are negative.
 * North latitudes are positive, South latitudes are negative
 * Lat and Long are in fractional degrees.
 *
 * Written by Chuck Gantz- chuck.gantz@globalstar.com
 */
inline void UTMtoLL(const double UTMEasting, const double UTMNorthing, const int UTMZone, const int hemi,
                           double& Lat, double& Long)
{
  double k0 = UTM_K0;
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double eccPrimeSquared;
  double e1 = (1 - sqrt(1 - eccSquared)) / (1 + sqrt(1 - eccSquared));
  double N1, T1, C1, R1, D, M;
  double LongOrigin;
  double mu, phi1, phi1Rad;
  double x, y;

  x = UTMEasting - 500000.0; //remove 500,000 meter offset for longitude
  y = UTMNorthing;

  if (hemi == SOUTHERN_HEMISPHERE) {
    y -= 10000000.0; //remove 10,000,000 meter offset used for southern hemisphere
  }

  LongOrigin = (UTMZone - 1) * 6 - 180 + 3;  //+3 puts origin in middle of zone

  eccPrimeSquared = (eccSquared) / (1 - eccSquared);

  M = y / k0;
  mu = M
      / (a * (1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 - 5 * eccSquared * eccSquared * eccSquared / 256));

  phi1Rad = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * sin(2 * mu)
      + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * sin(4 * mu) + (151 * e1 * e1 * e1 / 96) * sin(6 * mu);
  phi1 = rad2Deg(phi1Rad);

  N1 = a / sqrt(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad));
  T1 = tan(phi1Rad) * tan(phi1Rad);
  C1 = eccPrimeSquared * cos(phi1Rad) * cos(phi1Rad);
  R1 = a * (1 - eccSquared) / pow(1 - eccSquared * sin(phi1Rad) * sin(phi1Rad), 1.5);
  D = x / (N1 * k0);

  Lat = phi1Rad
      - (N1 * tan(phi1Rad) / R1)
          * (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * eccPrimeSquared) * D * D * D * D / 24
              + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * eccPrimeSquared - 3 * C1 * C1) * D * D * D * D * D * D
                  / 720);
  Lat = rad2Deg(Lat);

  Long = (D - (1 + 2 * T1 + C1) * D * D * D / 6
      + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * eccPrimeSquared + 24 * T1 * T1) * D * D * D * D * D / 120)
      / cos(phi1Rad);
  Long = LongOrigin + rad2Deg(Long);
}

/*! \brief Class to contain a lat/lon point
 *
 * This class represents a geographic point in lat/lon.  It also supports conversion to ENU, ECEF, and UTM
 * Cartesian coordinate systems.
 *
 */
class LatLon
{
public:
  /// Empty constructor initializes coordinates to zero
  LatLon()
  {
    lat_ = 0;
    lon_ = 0;
    alt_ = 0;
  }

  /*! \brief Initialize coordinates to equivalent ECEF point
   * @param[in] ecef ECEF coordinates contained in a tf::Vector3
   */
  LatLon(const tf::Vector3& ecef)
  {
    setEcef(ecef);
  }

  /*! \brief Directly initialize the coordinates with lat, lon, and alt.
   * @param[in] lat Latitude in degrees
   * @param[in] lon Longitude in degrees
   * @param[in] alt Altitude in meters
   */

  LatLon(double lat, double lon, double alt)
  {
    lat_ = lat;
    lon_ = lon;
    alt_ = alt;
  }

  /*! \brief Initialize coordinates using the data in the ROS message 'sensor_msgs::NavSatFix'.
   *
   * @param[in] fix
   */
  LatLon(const sensor_msgs::NavSatFix& fix)
  {
    lat_ = fix.latitude;
    lon_ = fix.longitude;
    alt_ = fix.altitude;
  }

  /*! \brief Returns the current latitude.
   * @return Latitude in degrees
   */
  double getLat()
  {
    return lat_;
  }

  /*! \brief Returns the current longitude.
   * @return Longitude in degrees
   */
  double getLon()
  {
    return lon_;
  }

  /*! \brief Returns the current altitude.
   * @return Altitude in meters
   */
  double getAlt()
  {
    return alt_;
  }

  /*! \brief Converts the current coordinates into ECEF
   *
   * @return ECEF coordinates as a tf::Vector3
   */
  tf::Vector3 ecef(){
    double ecef_x;
    double ecef_y;
    double ecef_z;

    LLtoECEF(deg2Rad(lat_), deg2Rad(lon_), alt_, ecef_x, ecef_y, ecef_z);
    return tf::Vector3(ecef_x, ecef_y, ecef_z);
  }

  /*! \brief Uses the current coordinates as a reference point, then
   * converts an ECEF point into ENU coordinates centered at the reference.
   *
   *
   */
  tf::Vector3 toEnu(const tf::Vector3& ecef){
    return getTransform()(ecef);
  }

  /*! \brief Computes and returns a transform from ECEF to an ENU frame centered at the current coordinates.
   * @return Translation and rotation represented in a tf::Transform
   */
  tf::Transform getTransform(){
    double s_lat = sin(deg2Rad(lat_));
    double c_lat = cos(deg2Rad(lat_));
    double s_lon = sin(deg2Rad(lon_));
    double c_lon = cos(deg2Rad(lon_));

    tf::Transform t(tf::Matrix3x3(-s_lon, -s_lat*c_lon, c_lat*c_lon, c_lon, -s_lat*s_lon, c_lat*s_lon, 0, c_lat, s_lat), ecef());

    return t.inverse();
  }

  /*! \brief Sets the current coordinates by converting ECEF to lat/lon
   * @param[in] ecef ECEF point expressed as a tf::Vector3.
   */
  void setEcef(const tf::Vector3& ecef){
    ECEFtoLL(ecef.x(), ecef.y(), ecef.z(), lat_, lon_, alt_);
  }
private:
  double lat_;
  double lon_;
  double alt_;
};

/*! \brief Class to contain a UTM point.
 *
 * This class represents a geographic point in Universal Transverse Mercator (UTM) coordinates.
 * Operators are overloaded to make converting to and from lat/lon more convenient.
 *
 */
class UTMCoords
{
public:
  /// Empty constructor initializes everything to zero.
  UTMCoords()
  {
    x_ = 0;
    y_ = 0;
    z_ = 0;
    zone_ = 0;
    hemi_ = UNDEFINED_HEMISPHERE;
  }

  /*! \brief Initialize UTM coordinates directly
   *
   * @param[in] x UTM Easting coordinate in meters
   * @param[in] y UTM Northing coordinate in meters
   * @param[in] z Altitude in meters
   * @param[in] zone UTM zone number
   * @param[in] hemi Hemisphere (1 = northern, -1 = southern)
   */
  UTMCoords(double x, double y, double z, int zone, int hemi)
  {
    x_ = x;
    y_ = y;
    z_ = z;
    zone_ = zone;
    hemi_ = hemi;
  }

  /*! \brief Initialize coordinates using the data in the ROS message 'sensor_msgs::NavSatFix'.
   *
   * @param[in] fix
   */
  UTMCoords(const sensor_msgs::NavSatFix& fix)
  {
    LLtoUTM(fix.latitude, fix.longitude, x_, y_, zone_, hemi_);
    if (std::isnan(fix.altitude)){
      z_ = 0.0;
    }else{
      z_ = fix.altitude;
    }
  }

  /*! \brief Initialize coordinates using an instance of the LatLon class
   *
   * @param[in] fix LatLon class instance
   */
  UTMCoords(LatLon fix)
  {
    LLtoUTM(fix.getLat(), fix.getLon(), x_, y_, zone_, hemi_);
    z_ = fix.getAlt();
  }

  /*! \brief Returns the current UTM Easting coordinate
   * @return UTM Easting in meters
   */
  double getX()
  {
    return x_;
  }
  /*! \brief Returns the current UTM Northing coordinate
   * @return UTM Northing in meters
   */
  double getY()
  {
    return y_;
  }
  /*! \brief Returns the current altitude
   * @return Altitude in meters
   */
  double getZ()
  {
    return z_;
  }
  /*! \brief Returns the current UTM zone
   * @return UTM zone number (1 - 60)
   */
  int getZone()
  {
    return zone_;
  }
  /*! \brief Returns the current hemisphere
   * @return UTM hemisphere (1 = northern, -1 = southern)
   */
  int getHemi()
  {
    return hemi_;
  }

  /*! \brief Set the UTM Easting directly.
   * @param[in] x UTM Easting in meters
   */
  void setX(double x)
  {
    x_ = x;
  }
  /*! \brief Set the UTM Northing directly.
   * @param[in] y UTM Easting in meters
   */
  void setY(double y)
  {
    y_ = y;
  }
  /*! \brief Set the altitude directly.
   * @param[in] z Altitude in meters
   */
  void setZ(double z)
  {
    z_ = z;
  }
  /*! \brief Set the UTM zone directly.
   * @param[in] zone UTM zone number (1 - 60)
   */
  void setZone(int zone)
  {
    zone_ = zone;
  }

  /*! \brief Set the hemisphere directly.
   * @param[in] hemi Hemisphere (1 = northern, -1 = southern)
   */
  void setHemi(int hemi)
  {
    hemi_ = hemi;
  }

  /// Return the current UTM coordinates as a tf::Vector3
  tf::Vector3 asVector3()
  {
    tf::Vector3 v(x_, y_, z_);
  }

  /*! \brief Subtract one UTMCoords class instance from another to get relative position vector
   * @return Relative position vector as a tf::Vector3
   */
  inline tf::Vector3 operator -(UTMCoords& utm2)
  {
    tf::Vector3 del_utm;
    if ((zone_ == utm2.getZone()) && (hemi_ == utm2.getHemi())) {
      del_utm.setX(x_ - utm2.getX());
      del_utm.setY(y_ - utm2.getY());
      del_utm.setZ(z_ - utm2.getZ());
    }
    return del_utm;
  }

  /*! \brief Add a relative position vector to current coordinates, and convert the result to lat/lon
   * @param local_coords Relative position vector from current UTM coordinates.
   * @return LatLon class instance representing the absolute position of 'local_coords'
   */
  inline LatLon operator +(tf::Vector3& local_coords)
  {
    double lat, lon;
    UTMtoLL(x_ + local_coords.getX(), y_ + local_coords.getY(), zone_, hemi_, lat, lon);
    return LatLon(lat, lon, z_ + local_coords.getZ());
  }

  inline LatLon operator +(const tf::Vector3& local_coords)
  {
    double lat, lon;
    UTMtoLL(x_ + local_coords.getX(), y_ + local_coords.getY(), zone_, hemi_, lat, lon);
    return LatLon(lat, lon, z_ + local_coords.getZ());
  }

private:
  double x_;
  double y_;
  double z_;
  int zone_;
  int hemi_;
};

#endif /* GPS_CONV_H_ */
