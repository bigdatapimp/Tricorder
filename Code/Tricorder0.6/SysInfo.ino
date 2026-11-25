// ------------- System Info ---------------
//
// Three pages (UP / DOWN = previous / next page, LEFT long = exit):
//   1/3  Memory and uptime  - free heap, uptime, loop rate (how responsive the main loop is)
//   2/3  GPS health         - satellites, HDOP, fix age, NMEA characters received / checksum failures
//   3/3  Device             - MCU, RAM / flash sizes, unique chip ID, firmware build date
//
// The TFT shows a bar graph of free RAM.
//
// "Free heap" is measured by asking malloc() for the biggest block it can give us (binary search,
// each probe is freed straight away). That is the largest single allocation that would succeed
// right now, which is the number that matters for running out of memory. Percent is relative to
// the nRF52840's 256 KB of RAM, so even a freshly booted sketch never shows 100%: the program's own
// variables and the stack take their share.

#define SYSINFO_PAGES  3
#define SYS_RAM_BYTES  262144UL     // nRF52840 RAM
#define SYS_FLASH_KB   1024         // nRF52840 flash

// Largest block malloc() will currently hand out, to within 512 bytes
uint32_t sysLargestFreeBlock() {
  uint32_t lo = 0;
  uint32_t hi = SYS_RAM_BYTES;
  while (hi - lo > 512) {
    uint32_t mid = (lo + hi) / 2;
    void* p = malloc(mid);
    if (p != NULL) {
      free(p);
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}

// The colour screen (LCARS look, see Lcars.ino): the page in the header's tag, the free RAM as a number and a 22-segment bar
// (red up to 20 %, amber up to 40 %, green above), the uptime, the loop rate and the biggest free block
#define SYS_SEGS 22

void sysDrawTftStatic() {
  lcFrame("SYSINFO", true);
  lcPill(8, 11, 20, lcLilac(), "RAM");
  lcPill(8, 33, 14, lcBlue(), "UP");
  lcPill(8, 43, 26, lcOrange(), "LOOP");
  lcPill(8, 53, 26, lcOk(), "HEAP");
  for (int i = 0; i < SYS_SEGS; i++) lcSeg(8 + i * 4, 21, 3, 9, lcPercentColor(i * 100 / SYS_SEGS), false);
}

void SystemInfo(int pressType) {
  static bool initialized = false;
  static int drawnPage = -1;
  static char shownUp[16], shownLoop[12], shownHeap[12];
  static int page = 0;
  static unsigned long lastRefresh = 0;
  static unsigned long loopCount = 0;
  static unsigned long loopWindowStart = 0;
  static unsigned long loopsPerSec = 0;
  static int drawnPct = -1;

  // --- Buttons ---
  if (pressType == LONG_PRESS && lastButton == 5) {      // Long Left: exit
    tft.fillScreen(BLACK);
    initialized = false;
    currentState = SYS_MENU;
    return;
  }
  if (pressType == SHORT_PRESS) {
    if (lastButton == 4) { page = (page + 1) % SYSINFO_PAGES; lastRefresh = 0; }                    // Down: next page
    if (lastButton == 2) { page = (page + SYSINFO_PAGES - 1) % SYSINFO_PAGES; lastRefresh = 0; }    // Up: previous page
  }

  // --- First pass after entering ---
  if (!initialized) {
    initialized = true;
    page = 0;
    lastRefresh = 0;
    loopCount = 0;
    loopWindowStart = millis();
    drawnPct = -1;

    drawnPage = -1;
    tftTextForget(shownUp, sizeof(shownUp));
    tftTextForget(shownLoop, sizeof(shownLoop));
    tftTextForget(shownHeap, sizeof(shownHeap));
    sysDrawTftStatic();
  }

  // --- Loop rate: how many times per second this function gets called ---
  loopCount++;
  unsigned long now = millis();
  if (now - loopWindowStart >= 1000) {
    loopsPerSec = loopCount * 1000UL / (now - loopWindowStart);
    loopCount = 0;
    loopWindowStart = now;
  }

  // --- Refresh twice a second (or immediately after a page change) ---
  if (now - lastRefresh < 500) return;
  lastRefresh = now;

  uint32_t freeBytes = sysLargestFreeBlock();
  int pct = (int)(freeBytes * 100UL / SYS_RAM_BYTES);

  // --- TFT: the RAM bar and number, only redrawn when the percentage changes ---
  if (pct != drawnPct) {
    drawnPct = pct;
    int lit = (pct * SYS_SEGS + 50) / 100;
    for (int i = 0; i < SYS_SEGS; i++) lcSeg(8 + i * 4, 21, 3, 9, lcPercentColor(i * 100 / SYS_SEGS), i < lit);
    char buf[16];
    snprintf(buf, sizeof(buf), "%3d%% FREE", pct);
    tft.setTextSize(1);
    tft.setTextColor(lcPercentColor(pct), BLACK);
    tft.setCursor(31, 11);
    tft.print(buf);
  }

  // --- TFT: the page, and the lines that change as time goes by (only the characters that differ are drawn) ---
  if (page != drawnPage) {
    drawnPage = page;
    char tag[8];
    snprintf(tag, sizeof(tag), "%d/%d", page + 1, SYSINFO_PAGES);
    lcSetTag(tag);
  }
  {
    unsigned long upSec = now / 1000UL;
    unsigned long upD = upSec / 86400UL;
    if (upD > 99) upD = 99;
    char buf[16];
    snprintf(buf, sizeof(buf), "%lud %02lu:%02lu:%02lu", upD, (upSec / 3600UL) % 24UL, (upSec / 60UL) % 60UL, upSec % 60UL);
    char padded[16];
    snprintf(padded, sizeof(padded), "%-12s", buf);
    tftTextDiff(shownUp, 24, 33, 1, padded, lcPeach(), BLACK);
    snprintf(buf, sizeof(buf), "%lu/s", loopsPerSec);
    snprintf(padded, sizeof(padded), "%-8s", buf);
    tftTextDiff(shownLoop, 38, 43, 1, padded, lcPeach(), BLACK);
    snprintf(buf, sizeof(buf), "%lu KB", (unsigned long)(freeBytes / 1024UL));
    snprintf(padded, sizeof(padded), "%-8s", buf);
    tftTextDiff(shownHeap, 38, 53, 1, padded, lcPeach(), BLACK);
  }

  // --- OLED ---
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 9);
  display.print(F("SYSTEM INFO"));
  display.setCursor(104, 9);
  display.print(page + 1);
  display.print(F("/"));
  display.print(SYSINFO_PAGES);

  char line[24];

  if (page == 0) {
    // ---- Memory and uptime ----
    display.setCursor(0, 20);
    snprintf(line, sizeof(line), "Heap free:%4lu KB", (unsigned long)(freeBytes / 1024UL));
    display.print(line);

    display.setCursor(0, 30);
    snprintf(line, sizeof(line), "of RAM:   %4d %%", pct);
    display.print(line);

    unsigned long secs = now / 1000UL;
    unsigned long d = secs / 86400UL;
    unsigned long h = (secs / 3600UL) % 24UL;
    unsigned long m = (secs / 60UL) % 60UL;
    unsigned long s = secs % 60UL;
    display.setCursor(0, 40);
    snprintf(line, sizeof(line), "Up: %lud %02lu:%02lu:%02lu", d, h, m, s);
    display.print(line);

    display.setCursor(0, 50);
    snprintf(line, sizeof(line), "Loop: %lu /s", loopsPerSec);
    display.print(line);

  } else if (page == 1) {
    // ---- GPS health ----
    display.setCursor(0, 20);
    display.print(F("Sats:"));
    if (gps.satellites.isValid()) display.print((int)gps.satellites.value()); else display.print(F("--"));
    display.print(F("  HDOP:"));
    if (gps.hdop.isValid()) display.print(gps.hdop.hdop(), 1); else display.print(F("--"));

    display.setCursor(0, 30);
    display.print(F("Fix: "));
    if (gps.location.isValid()) {
      display.print(F("Yes  Age:"));
      display.print(gps.location.age() / 1000UL);
      display.print(F("s"));
    } else {
      display.print(F("No"));
    }

    display.setCursor(0, 40);
    snprintf(line, sizeof(line), "NMEA chars:%lu", (unsigned long)gps.charsProcessed());
    display.print(line);

    display.setCursor(0, 50);
    snprintf(line, sizeof(line), "Bad cksum:%lu", (unsigned long)gps.failedChecksum());
    display.print(line);

  } else {
    // ---- Device ----
    display.setCursor(0, 20);
    display.print(F("FW v"));
    display.print(FWver, 2);
    display.print(F("  nRF52840"));

    display.setCursor(0, 30);
    snprintf(line, sizeof(line), "RAM %luK Flash %dK", SYS_RAM_BYTES / 1024UL, SYS_FLASH_KB);
    display.print(line);

    display.setCursor(0, 40);
    display.print(F("ID:"));
#ifdef NRF_FICR
    snprintf(line, sizeof(line), "%08lX%08lX", (unsigned long)NRF_FICR->DEVICEID[1], (unsigned long)NRF_FICR->DEVICEID[0]);
    display.print(line);
#else
    display.print(F("n/a"));
#endif

    display.setCursor(0, 50);
    display.print(F("Built:"));
    display.print(__DATE__);
  }

  display.display();
}
