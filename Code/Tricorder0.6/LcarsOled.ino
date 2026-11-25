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
