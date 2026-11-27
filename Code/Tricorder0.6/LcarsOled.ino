// ------------- LCARS look for the OLED (128 x 64, one colour) -------------
//
// The same look as the colour screen (see Lcars.ino) in black and white. The OLED has a yellow strip across the top (rows 0 to
// 15) and blue below it: the header is drawn in the yellow strip and the screen's own content in the blue part, which is
// x 9..127, y 18..63 (lines of up to 19 characters, 9 pixels apart). Where the colour screen uses colours, this uses fills:
// solid, hollow or dotted.
//
//   display.clearDisplay(); lcoFrame("TITLE", "TAG"); ... your content ...; display.display();
//
//   lcoFrame(title, tag)                 the header (the tag block is optional: pass NULL or "") and the bar down the left edge
//   lcoPill(x, y, w, h, text)            a solid white block with black letters
//   lcoOutline(x, y, w, h, text)         a hollow block with white letters
//   lcoBar(x, y, n, lit, w, h, pitch)    n segments, the first 'lit' solid and the rest hollow
//   lcoDrawMenu(title, names, count, cursor)   a whole menu screen: the selected item is a solid block, the others hollow
//   lcoDrawMainMenu()                    the main menu (for handleMainMenu in the main sketch)
#define LCO_BAND_H   16      // height of the yellow strip: the header lives inside it
#define LCO_CONTENT_X 9
#define LCO_CONTENT_Y 18

void lcoFrame(const char* title, const char* tag) {
  int tw = 6 * (int)strlen(title) + 12;                  // the title block, rounded at both ends
  display.fillRoundRect(0, 0, tw, LCO_BAND_H - 1, 6, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(8, 4);
  display.print(title);
  int x = tw + 3;
  if (tag != NULL && tag[0] != '\0') {                   // a small block with a few characters
    int w = 6 * (int)strlen(tag) + 6;
    display.fillRect(x, 0, w, LCO_BAND_H - 1, SSD1306_WHITE);
    display.setCursor(x + 3, 4);
    display.print(tag);
    x += w + 3;
  } else {                                               // a hollow block
    display.drawRect(x, 0, 14, LCO_BAND_H - 1, SSD1306_WHITE);
    x += 17;
  }
  if (x < 120) display.fillRoundRect(x, 0, 128 - x, LCO_BAND_H - 1, 6, SSD1306_WHITE);

  // the bar down the left edge: solid, hollow, dotted
  display.fillRect(0, 7, 5, 24, SSD1306_WHITE);
  display.drawRect(0, 32, 5, 16, SSD1306_WHITE);
  for (int y = 49; y < 64; y++) {
    for (int px = 0; px < 5; px++) {
      if (((px + y) & 1) == 0) display.drawPixel(px, y, SSD1306_WHITE);
    }
  }

  // the inner curve where the bar meets the header
  display.fillRect(5, LCO_BAND_H - 1, 3, 3, SSD1306_WHITE);
  display.fillCircle(8, LCO_BAND_H + 2, 3, SSD1306_BLACK);
}

void lcoPill(int x, int y, int w, int h, const char* text) {
  display.fillRoundRect(x, y, w, h, 3, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_BLACK);
  display.setCursor(x + (w - 6 * (int)strlen(text)) / 2, y + (h - 7) / 2);
  display.print(text);
}

void lcoOutline(int x, int y, int w, int h, const char* text) {
  display.drawRoundRect(x, y, w, h, 3, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x + (w - 6 * (int)strlen(text)) / 2, y + (h - 7) / 2);
  display.print(text);
}

void lcoBar(int x, int y, int n, int lit, int w, int h, int pitch) {
  for (int i = 0; i < n; i++) {
    if (i < lit) display.fillRect(x + i * pitch, y, w, h, SSD1306_WHITE);
    else         display.drawRect(x + i * pitch, y, w, h, SSD1306_WHITE);
  }
}

// A menu: one block per item, the selected one solid with black letters, the others hollow. Up to 4 items are 10 pixels
// high, 5 are 8 pixels high.
void lcoDrawMenu(const char* title, const char* const* names, int count, int cursor) {
  display.clearDisplay();
  lcoFrame(title, NULL);
  int pitch = (count <= 4) ? 11 : 9;
  int h = pitch - 1;
  for (int i = 0; i < count; i++) {
    int y = LCO_CONTENT_Y + i * pitch;
    if (i == cursor) display.fillRoundRect(LCO_CONTENT_X, y, 118, h, 3, SSD1306_WHITE);
    else             display.drawRoundRect(LCO_CONTENT_X, y, 118, h, 3, SSD1306_WHITE);
    display.setTextSize(1);
    display.setTextColor(i == cursor ? SSD1306_BLACK : SSD1306_WHITE);
    display.setCursor(LCO_CONTENT_X + 6, y + (h - 7) / 2);
    display.print(names[i]);
  }
  display.display();
}

void lcoDrawMainMenu() {
  lcoDrawMenu("MAIN MENU", lcMainNames, 5, mainMenuCursor);
}

// ---- rows of text, blocks and whole screens ----
// The content lines are 9 pixels apart from y = 18 (rows 0 to 4). Text is at most 19 characters a line.

int lcoRowY(int row) {
  return LCO_CONTENT_Y + 9 * row;
}

// A line of text in a row
void lcoText(int row, const char* text) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(LCO_CONTENT_X, lcoRowY(row));
  display.print(text);
}

// Text at any position, cut to 'maxChars' characters
void lcoTextAt(int x, int y, const char* text, int maxChars) {
  char buf[32];
  if (maxChars > 30) maxChars = 30;
  strncpy(buf, text, maxChars);
  buf[maxChars] = '\0';
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.print(buf);
}

// A row with a solid label block and its value after it
void lcoRow(int row, const char* label, const char* value) {
  int w = 6 * (int)strlen(label) + 6;
  lcoPill(LCO_CONTENT_X, lcoRowY(row), w, 8, label);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(LCO_CONTENT_X + w + 4, lcoRowY(row));
  display.print(value);
}

// A wide block across the content area, solid with black letters or hollow with white ones
void lcoBlock(int y, int h, const char* text, bool solid) {
  if (solid) lcoPill(LCO_CONTENT_X, y, 118, h, text);
  else       lcoOutline(LCO_CONTENT_X, y, 118, h, text);
}

// Text wrapped at the spaces into lines of at most 'maxChars' characters, at most 'maxRows' of them, 9 pixels apart from (x, y)
int lcoWrap(int x, int y, int maxChars, int maxRows, const char* text) {
  char line[24];
  int n = 0, rows = 0;
  if (maxChars > 20) maxChars = 20;
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  const char* p = text;
  while (*p != '\0' && rows < maxRows) {
    while (*p == ' ') p++;
    if (*p == '\0') break;
    int len = 0;
    while (p[len] != '\0' && p[len] != ' ') len++;
    int rest = len;
    if (len > maxChars) len = maxChars;
    if (n > 0 && n + 1 + len > maxChars) {
      line[n] = '\0';
      display.setCursor(x, y + rows * 9);
      display.print(line);
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
    display.setCursor(x, y + rows * 9);
    display.print(line);
    rows++;
  }
  return rows;
}

// A message screen: the header and up to five lines of text (NULL for none)
void lcoMessage(const char* title, const char* tag, const char* l1, const char* l2, const char* l3, const char* l4, const char* l5) {
  display.clearDisplay();
  lcoFrame(title, tag);
  const char* lines[5] = {l1, l2, l3, l4, l5};
  for (int i = 0; i < 5; i++) {
    if (lines[i] != NULL) lcoText(i, lines[i]);
  }
  display.display();
}

// A screen with one wide block in the middle: solid or hollow
void lcoBlockScreen(const char* title, const char* tag, const char* text, bool solid) {
  display.clearDisplay();
  lcoFrame(title, tag);
  lcoBlock(28, 14, text, solid);
  display.display();
}

// The start-up screen: "POWERING ON" with a bar that fills over 'totalMs' milliseconds (this replaces the delay)
void lcoPowerOn(const char* version, unsigned long totalMs) {
  const int segs = 22;
  for (int i = 0; i <= segs; i++) {
    display.clearDisplay();
    lcoFrame("TRICORDER", version);
    lcoBlock(22, 12, "POWERING ON", true);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(LCO_CONTENT_X + (118 - 6 * 8) / 2, 39);
    display.print("STAND BY");
    lcoBar(LCO_CONTENT_X, 52, segs, i, 3, 7, 5);
    display.display();
    delay(totalMs / (segs + 1));
  }
}

// A calibration screen: the tag says what is being calibrated, up to three lines of text, a progress bar (0 to 100 percent)
// and a line under it
void lcoCalibrating(const char* tag, const char* l1, const char* l2, const char* l3, int percent, const char* bottom) {
  display.clearDisplay();
  lcoFrame("CALIBRATING", tag);
  if (l1 != NULL) lcoText(0, l1);
  if (l2 != NULL) lcoText(1, l2);
  if (l3 != NULL) lcoText(2, l3);
  int lit = percent * 22 / 100;
  if (lit < 0) lit = 0;
  if (lit > 22) lit = 22;
  lcoBar(LCO_CONTENT_X, 46, 22, lit, 3, 5, 5);
  if (bottom != NULL) lcoTextAt(LCO_CONTENT_X, 54, bottom, 19);
  display.display();
}
