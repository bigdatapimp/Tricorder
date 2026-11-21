// ------------- MGRS / UTM conversion ---------------
//
// Converts a WGS84 latitude/longitude (as given by the GPS) into UTM and then into an MGRS reference
// to 1 metre, e.g.  16S EG 12345 67890
//   16S   = grid zone designator (UTM zone 16, latitude band S)
//   EG    = 100 km square letters (column, row)
//   12345 = easting  in metres within that 100 km square
//   67890 = northing in metres within that 100 km square
// Values are truncated, not rounded, as the MGRS standard specifies.
//
// Covers latitudes 80 S to 84 N (the normal UTM/MGRS range, including the Norway and Svalbard zone
// exceptions). The polar caps use a different system (UPS) which is not implemented: the function
// returns false there.
//
// The projection uses the Kruger series to 6th order (the same method as Karney's 2011 paper), which is
// accurate to well under a millimetre across the whole UTM range. It relies on 'double' being 64-bit.
//
// (No custom struct types in these signatures on purpose: the Arduino IDE generates function prototypes
//  above all the sketch code, where a struct from this tab wouldn't be declared yet.)

#define MGRS_DEG2RAD 0.017453292519943295

// UTM zone number for a position, including the Norway (32V) and Svalbard (31X, 33X, 35X, 37X) exceptions
int utmZoneFor(double lat, double lon) {
  while (lon >= 180.0) lon -= 360.0;
  while (lon < -180.0) lon += 360.0;
  int zone = (int)floor((lon + 180.0) / 6.0) + 1;
  if (zone < 1) zone = 1;
  if (zone > 60) zone = 60;

  if (lat >= 56.0 && lat < 64.0 && lon >= 3.0 && lon < 12.0) zone = 32;   // southwest Norway
  if (lat >= 72.0) {                                                      // Svalbard
    if (lon >= 0.0 && lon < 9.0) zone = 31;
    else if (lon >= 9.0 && lon < 21.0) zone = 33;
    else if (lon >= 21.0 && lon < 33.0) zone = 35;
    else if (lon >= 33.0 && lon < 42.0) zone = 37;
  }
  return zone;
}

// Latitude/longitude -> UTM easting and northing (metres) in the given zone.
// 'easting' includes the 500 000 m false easting; 'northing' includes 10 000 000 m for the southern hemisphere.
void utmFromLatLon(double lat, double lon, int zone, double* easting, double* northing) {
  const double a  = 6378137.0;                  // WGS84 semi-major axis
  const double f  = 1.0 / 298.257223563;        // WGS84 flattening
  const double k0 = 0.9996;                     // UTM scale factor on the central meridian

  const double e  = sqrt(f * (2.0 - f));        // eccentricity
  const double n  = f / (2.0 - f);              // third flattening
  const double n2 = n * n, n3 = n2 * n, n4 = n3 * n, n5 = n4 * n, n6 = n5 * n;

  double lon0 = ((zone - 1) * 6 - 180 + 3) * MGRS_DEG2RAD;      // central meridian of the zone
  double phi = lat * MGRS_DEG2RAD;
  double lam = lon * MGRS_DEG2RAD - lon0;
  while (lam > PI) lam -= 2.0 * PI;
  while (lam < -PI) lam += 2.0 * PI;

  double cosL = cos(lam), sinL = sin(lam);
  double tau = tan(phi);
  double sigma = sinh(e * atanh(e * tau / sqrt(1.0 + tau * tau)));
  double tauP = tau * sqrt(1.0 + sigma * sigma) - sigma * sqrt(1.0 + tau * tau);
  double xiP  = atan2(tauP, cosL);
  double etaP = asinh(sinL / sqrt(tauP * tauP + cosL * cosL));

  double A = a / (1.0 + n) * (1.0 + n2 / 4.0 + n4 / 64.0 + n6 / 256.0);

  double alpha[7];
  alpha[1] = n / 2.0 - 2.0 * n2 / 3.0 + 5.0 * n3 / 16.0 + 41.0 * n4 / 180.0 - 127.0 * n5 / 288.0 + 7891.0 * n6 / 37800.0;
  alpha[2] = 13.0 * n2 / 48.0 - 3.0 * n3 / 5.0 + 557.0 * n4 / 1440.0 + 281.0 * n5 / 630.0 - 1983433.0 * n6 / 1935360.0;
  alpha[3] = 61.0 * n3 / 240.0 - 103.0 * n4 / 140.0 + 15061.0 * n5 / 26880.0 + 167603.0 * n6 / 181440.0;
  alpha[4] = 49561.0 * n4 / 161280.0 - 179.0 * n5 / 168.0 + 6601661.0 * n6 / 7257600.0;
  alpha[5] = 34729.0 * n5 / 80640.0 - 3418889.0 * n6 / 1995840.0;
  alpha[6] = 212378941.0 * n6 / 319334400.0;

  double xi = xiP, eta = etaP;
  for (int j = 1; j <= 6; j++) {
    xi  += alpha[j] * sin(2.0 * j * xiP) * cosh(2.0 * j * etaP);
    eta += alpha[j] * cos(2.0 * j * xiP) * sinh(2.0 * j * etaP);
  }

  *easting  = 500000.0 + k0 * A * eta;
  *northing = k0 * A * xi;
  if (lat < 0.0) *northing += 10000000.0;
}

// Latitude/longitude -> MGRS parts. Returns false if the position is outside 80 S .. 84 N (or not a number).
//   zone       1..60
//   band       latitude band letter C..X
//   colLetter / rowLetter   100 km square identification
//   easting / northing      metres within that 100 km square (0..99999)
bool mgrsFromLatLon(double lat, double lon, int* zone, char* band, char* colLetter, char* rowLetter,
                    long* easting, long* northing) {
  if (!(lat >= -80.0 && lat <= 84.0)) return false;     // also rejects NaN
  if (!(lon >= -540.0 && lon <= 540.0)) return false;
  while (lon >= 180.0) lon -= 360.0;
  while (lon < -180.0) lon += 360.0;

  int z = utmZoneFor(lat, lon);
  double e, n;
  utmFromLatLon(lat, lon, z, &e, &n);

  long eInt = (long)floor(e);                           // MGRS truncates
  long nInt = (long)floor(n);

  // Latitude band: 8 degree bands from 80 S, letters skip I and O; the last band (X) is 12 degrees (72-84 N)
  const char* bands = "CDEFGHJKLMNPQRSTUVWX";
  int bandIdx = (int)floor((lat + 80.0) / 8.0);
  if (bandIdx > 19) bandIdx = 19;
  if (bandIdx < 0) bandIdx = 0;

  // 100 km column letter: the lettering repeats every 3 zones
  const char* colSets[3] = {"ABCDEFGH", "JKLMNPQR", "STUVWXYZ"};
  int colIdx = (int)(eInt / 100000L) - 1;
  if (colIdx < 0 || colIdx > 7) return false;

  // 100 km row letter: A..V (no I, O), repeating every 2 000 km, shifted by 5 letters in even zones
  const char* rows = "ABCDEFGHJKLMNPQRSTUV";
  int rowIdx = (int)((nInt / 100000L) % 20L);
  if (z % 2 == 0) rowIdx = (rowIdx + 5) % 20;

  *zone = z;
  *band = bands[bandIdx];
  *colLetter = colSets[(z - 1) % 3][colIdx];
  *rowLetter = rows[rowIdx];
  *easting = eInt % 100000L;
  *northing = nInt % 100000L;
  return true;
}
