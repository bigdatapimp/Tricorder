// ------------- LCARS look for the colour screen (96 x 64) -------------
//
// Shared by the dB Meter, Spectrum, Metal Detector and EMF Meter screens. The colours are the same as the phone app's.
//
// The frame: a title block across the top with a rounded left end, bending down into a segmented bar along the left edge
// (the LCARS "elbow"). The free area for a screen's own content is x 8..95, y 10..63.
//
//   lcFrame("TITLE", true)           clears the screen and draws the frame; 'true' adds a small tag block to the right
//                                    of the title, for up to 4 characters
//   lcSetTag("G+10")                 writes those characters (after lcFrame(..., true))
//   lcPill(x, y, w, colour, "TEXT")  a rounded label block, 8 pixels high, with black text
//   lcSeg(x, y, w, h, colour, lit)   one segment of a bar: in its colour when lit, the same colour dimmed when not
//   lcHeat(step, steps)              blue (low) -> violet -> red (high) colours for a bar of 'steps' segments
//
// The colours are functions (lcOrange() and so on), so every tab can use them whatever order the tabs are compiled in.

uint16_t lcRgb(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3);
}

uint16_t lcOrange() { return lcRgb(255, 153, 0); }
uint16_t lcPeach()  { return lcRgb(255, 204, 153); }
uint16_t lcLilac()  { return lcRgb(204, 153, 204); }
uint16_t lcBlue()   { return lcRgb(153, 153, 255); }
uint16_t lcGold()   { return lcRgb(255, 204, 102); }
uint16_t lcOk()     { return lcRgb(102, 221, 170); }
uint16_t lcAlert()  { return lcRgb(255, 85, 68); }
uint16_t lcGrey()   { return lcRgb(140, 140, 160); }

// A colour at one fifth of its brightness: a bar segment that is not lit
uint16_t lcDim(uint16_t c) {
  int r = ((c >> 11) & 0x1F) * 255 / 31;
  int g = ((c >> 5) & 0x3F) * 255 / 63;
  int b = (c & 0x1F) * 255 / 31;
  return lcRgb((uint8_t)(r / 5), (uint8_t)(g / 5), (uint8_t)(b / 5));
}

// Blue at the low end, violet in the middle, red at the high end (the phone app's temperature colours)
uint16_t lcHeat(int step, int steps) {
  float p = (steps > 1) ? (float)step / (float)(steps - 1) : 0.0;
  float r, g, b;
  if (p < 0.5) {
    float t = p * 2.0;
    r = 61.0 + (155.0 - 61.0) * t;
    g = 123.0 + (91.0 - 123.0) * t;
    b = 255.0 + (232.0 - 255.0) * t;
  } else {
    float t = (p - 0.5) * 2.0;
    r = 155.0 + (255.0 - 155.0) * t;
    g = 91.0 + (69.0 - 91.0) * t;
    b = 232.0 + (56.0 - 232.0) * t;
  }
  return lcRgb((uint8_t)(r + 0.5), (uint8_t)(g + 0.5), (uint8_t)(b + 0.5));
}

void lcSeg(int x, int y, int w, int h, uint16_t color, bool lit) {
  tft.fillRect(x, y, w, h, lit ? color : lcDim(color));
}

// A rounded block with black text, centred in it. The text cell is 8 pixels high, the same as the block.
void lcPill(int x, int y, int w, uint16_t color, const char* text) {
  tft.fillRoundRect(x, y, w, 8, 3, color);
  tft.setTextSize(1);
  tft.setTextColor(BLACK, color);
  tft.setCursor(x + (w - 6 * (int)strlen(text)) / 2, y);
  tft.print(text);
}

int lcTagX = 0;                      // where the header's tag block starts (0 = this frame has none)

void lcFrame(const char* title, bool withTag) {
  uint16_t o = lcOrange();
  tft.fillScreen(BLACK);

  // the title block, rounded at both ends: its left end is the outer corner of the elbow
  int titleEnd = 8 + 6 * (int)strlen(title) + 3;
  tft.fillRoundRect(0, 0, titleEnd, 9, 4, o);
  tft.setTextSize(1);
  tft.setTextColor(BLACK, o);
  tft.setCursor(8, 1);
  tft.print(title);

  // more blocks along the header: a tag block for a few characters, or a plain one, then a closing block to the edge
  int x = titleEnd + 2;
  if (withTag) {
    lcTagX = x;
    tft.fillRect(x, 0, 28, 9, lcLilac());
    x += 30;
  } else {
    lcTagX = 0;
    tft.fillRect(x, 0, 14, 9, lcLilac());
    x += 16;
  }
  if (x < 94) tft.fillRoundRect(x, 0, 96 - x, 9, 4, lcBlue());

  // the bar down the left edge, in three segments
  tft.fillRect(0, 4, 5, 22, o);
  tft.fillRect(0, 27, 5, 16, lcLilac());
  tft.fillRect(0, 44, 5, 20, lcBlue());

  // the inner curve where the bar meets the title block
  tft.fillRect(5, 9, 3, 3, o);
  tft.fillCircle(8, 12, 3, BLACK);
}

// Up to 4 characters in the header's tag block (padded, so it replaces what was there)
void lcSetTag(const char* text) {
  if (lcTagX == 0) return;
  char buf[5] = {' ', ' ', ' ', ' ', 0};
  for (int i = 0; i < 4 && text[i] != '\0'; i++) buf[i] = text[i];
  tft.setTextSize(1);
  tft.setTextColor(BLACK, lcLilac());
  tft.setCursor(lcTagX + 2, 1);
  tft.print(buf);
}

// A wide rounded block with centred black text of any size (size 1 = 6 x 8 pixels a character, 2 = 12 x 16, ...)
void lcBigPill(int x, int y, int w, int h, uint16_t color, const char* text, int size) {
  tft.fillRoundRect(x, y, w, h, (h >= 16) ? 5 : 3, color);
  tft.setTextSize(size);
  tft.setTextColor(BLACK, color);
  tft.setCursor(x + (w - 6 * size * (int)strlen(text)) / 2, y + (h - 8 * size) / 2);
  tft.print(text);
}

// Text wrapped at the spaces into lines of at most 'maxChars' characters, at most 'maxRows' of them, 8 pixels apart,
// starting at (x, y). A word longer than a line is cut. Returns the number of lines used.
int lcWrap(int x, int y, int maxChars, int maxRows, const char* text, uint16_t color) {
  char line[24];
  int n = 0, rows = 0;
  if (maxChars > 20) maxChars = 20;
  tft.setTextSize(1);
  tft.setTextColor(color, BLACK);
  const char* p = text;
  while (*p != '\0' && rows < maxRows) {
    while (*p == ' ') p++;
    if (*p == '\0') break;
    int len = 0;
    while (p[len] != '\0' && p[len] != ' ') len++;
    int rest = len;
    if (len > maxChars) len = maxChars;
    if (n > 0 && n + 1 + len > maxChars) {               // it does not fit on this line: start the next one
      line[n] = '\0';
      tft.setCursor(x, y + rows * 8);
      tft.print(line);
      rows++;
      n = 0;
      if (rows >= maxRows) return rows;
    }
    if (n > 0) line[n++] = ' ';
    for (int i = 0; i < len; i++) line[n++] = p[i];
    p += rest;
  }
  if (n > 0 && rows < maxRows) {
    line[n] = '\0';
    tft.setCursor(x, y + rows * 8);
    tft.print(line);
    rows++;
  }
  return rows;
}

// Green above 40 %, amber above 20 %, red below: for a battery or a free-memory bar
uint16_t lcPercentColor(int pct) {
  return (pct > 40) ? lcOk() : ((pct > 20) ? lcGold() : lcAlert());
}

// The start-up screen: the name, the version and the author text (up to two lines), in the frame
void lcSplash(const char* version, const char* author) {
  lcFrame("TRICORDER", false);
  lcBigPill(8, 13, 88, 18, lcOrange(), version, 2);
  for (int i = 0; i < 22; i++) lcSeg(8 + i * 4, 36, 3, 6, lcHeat(i, 22), true);
  lcWrap(8, 47, 14, 2, author, lcPeach());          // the author text wraps onto at most two lines of 14 characters
}
