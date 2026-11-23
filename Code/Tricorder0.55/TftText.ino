// ------------- Shared TFT text helper ---------------
//
// Text on the colour TFT is slow (every pixel is a separate SPI write), so redrawing a whole line every
// time makes the loop sluggish and the text flicker. This draws only the characters that differ from what
// is already on screen. 'shown' is a buffer (at least as long as the longest text + 1) that remembers what
// is currently drawn; clear it with tftTextForget() whenever the screen area has been wiped.
//
// Always pass fixed-width text (pad with spaces) so a shorter string fully covers a longer one.

void tftTextDiff(char* shown, int x, int y, int size, const char* text, uint16_t fg, uint16_t bg) {
  tft.setTextSize(size);
  tft.setTextColor(fg, bg);
  for (int i = 0; text[i] != '\0'; i++) {
    if (shown[i] != text[i]) {
      tft.setCursor(x + i * 6 * size, y);
      tft.write(text[i]);
      shown[i] = text[i];
    }
  }
}

void tftTextForget(char* shown, int length) {
  memset(shown, 0, length);
}
