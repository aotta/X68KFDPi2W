// disk_image.h - disk image formats for X68KFDPi2W
//
// Sector images are turned into MFM flux track by track, starting from a list
// of sectors (C/H/R/N, size, position in the file). The same list is used to
// find which sector the X68000 is writing and where to store it in the file.
//
// Supported:
//   RAW  .HDM .XDF ...  plain sector dump, geometry from the file size
//   D88  .D88           2HD media, any sector layout (sizes, numbering,
//                       sector count per track, deleted data marks)
//   DIM  .DIM           X68000 DIFC.X images: 2HD, 2HS, 2HC, 2HDE, 26x256
//                       (layouts as in MAME's dim_dsk.cpp)
//   HFE  .HFE           read only, handled by the sketch (raw bitstream)
//
// Requires mfm_impl.h (Adafruit_Floppy) to be included first. IMG_LOG(...)
// can be defined by the includer to print diagnostics.
#pragma once
#include <stdint.h>
#include <string.h>
#include <strings.h>

#ifndef IMG_LOG
#define IMG_LOG(...) ((void)0)
#endif

enum { IMG_NONE = 0, IMG_RAW, IMG_D88, IMG_HFE };
enum { IMG_MAX_SECT = 32, D88_MAX_TRACKS = 164 };

struct img_sect_t {
  uint8_t c, h, r, n;  // ID field
  uint8_t deleted;     // 1 = deleted data mark (F8)
  uint16_t size;       // data bytes stored in the image
  uint16_t buf_off;    // offset of the data in the track buffer
  uint32_t file_off;   // offset of the data in the image file
  uint16_t idam_pos;   // byte offset of the ID mark in the compact flux
};

struct img_side_t {
  int track;           // -1 = not loaded
  uint8_t count;
  img_sect_t s[IMG_MAX_SECT];
};

struct img_raw_geom_t {
  uint32_t size;
  uint8_t cyls, sides, sectors, n;
  const char *name;
};

// Raw sector images recognised by their size (all 2HD, 500kbit/s, 360rpm)
static const img_raw_geom_t img_raw_geoms[] = {
    {1261568, 77, 2, 8, 3, "2HD 1232KB"}, // X68000 (HDM, XDF)
    {1228800, 80, 2, 15, 2, "2HC 1200KB"}, // PC-98 / PC AT 1.2MB
};

struct img_t {
  uint8_t type;
  bool wp;                       // image write protected (D88 flag)
  // IMG_RAW (also used for DIM): sectors in C/H/R order from data_off
  img_raw_geom_t geom;
  uint32_t data_off;
  bool r_2hs;                    // 2HS numbering: R = 10..18 (1 for the first
                                 // sector of cylinder 0)
  const char *fmt_name;          // shown on the web page
  uint32_t d88_hdr_end;          // IMG_D88: end of the track table
  img_side_t side[2];            // sectors of the sides currently loaded
};

static inline bool img_has_ext(const char *name, const char *ext) {
  size_t l = strlen(name), e = strlen(ext);
  return l >= e && !strcasecmp(name + l - e, ext);
}

static inline uint16_t img_le16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static inline uint32_t img_le32(const uint8_t *p) {
  return p[0] | (p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// Recognise the image. Returns false (with a message) if not supported.
template <class F> bool img_open(img_t &im, F &f, const char *name) {
  memset(&im, 0, sizeof(im));
  im.side[0].track = im.side[1].track = -1;
  if (img_has_ext(name, ".HFE")) {
    im.type = IMG_HFE;
    return true;
  }
  if (img_has_ext(name, ".D88") || img_has_ext(name, ".D68") ||
      img_has_ext(name, ".88D")) {
    uint8_t h[0x20];
    if (!f.seek(0) || f.read(h, sizeof(h)) != (int)sizeof(h)) {
      IMG_LOG("D88: cannot read header\n");
      return false;
    }
    if (h[0x1b] != 0x20) {
      IMG_LOG("D88: media type %02X not supported (2HD only)\n", h[0x1b]);
      return false;
    }
    // the track table ends where the first track starts
    uint32_t hdr_end = 0x20 + 4 * D88_MAX_TRACKS;
    for (int t = 0; t < D88_MAX_TRACKS; t++) {
      uint8_t o[4];
      if (!f.seek(0x20 + 4 * t) || f.read(o, 4) != 4)
        break;
      uint32_t off = img_le32(o);
      if (off && off < hdr_end)
        hdr_end = off;
      if ((uint32_t)(0x20 + 4 * (t + 1)) >= hdr_end)
        break;
    }
    im.type = IMG_D88;
    im.wp = h[0x1a] != 0;
    im.d88_hdr_end = hdr_end;
    return true;
  }
  if (img_has_ext(name, ".DIM")) {
    uint8_t h[0x100];
    if (!f.seek(0) || f.read(h, sizeof(h)) != (int)sizeof(h)) {
      IMG_LOG("DIM: cannot read header\n");
      return false;
    }
    if (memcmp(h + 0xab, "DIFC HEADER", 11))
      IMG_LOG("DIM: no DIFC signature, trying anyway\n");
    uint8_t spt, n;
    const char *fname;
    switch (h[0]) {
    case 0: spt = 8; n = 3; fname = "DIM 2HD"; break;
    case 1: spt = 9; n = 3; fname = "DIM 2HS"; im.r_2hs = true; break;
    case 2: spt = 15; n = 2; fname = "DIM 2HC"; break;
    case 3: spt = 9; n = 3; fname = "DIM 2HDE"; break;
    case 17: spt = 26; n = 1; fname = "DIM 26x256"; break;
    default:
      // 9 = 2HQ (1.44MB): needs a 300rpm drive
      IMG_LOG("DIM: media type %d not supported\n", h[0]);
      return false;
    }
    uint32_t track_bytes = (uint32_t)2 * spt * (128 << n);
    uint32_t cyls = (f.fileSize() - 0x100) / track_bytes;
    if (cyls > 82) cyls = 82;
    im.type = IMG_RAW;
    im.geom = {f.fileSize(), (uint8_t)cyls, 2, spt, n, fname};
    im.data_off = 0x100;
    im.fmt_name = fname;
    return true;
  }
  for (const auto &g : img_raw_geoms) {
    if (f.fileSize() == g.size) {
      im.type = IMG_RAW;
      im.geom = g;
      im.data_off = 0;
      im.fmt_name = g.name;
      return true;
    }
  }
  IMG_LOG("Unknown image size %lu\n", (unsigned long)f.fileSize());
  return false;
}

// Read the sector list and the data of one side into buf.
// A missing/unformatted track gives an empty list (no sectors on the track).
template <class F>
bool img_load_side(img_t &im, F &f, int track, int side, uint8_t *buf,
                   size_t bufsize) {
  img_side_t &L = im.side[side];
  L.track = track;
  L.count = 0;
  if (im.type == IMG_RAW) {
    const img_raw_geom_t *g = &im.geom;
    if (track >= g->cyls || side >= g->sides)
      return true;
    uint16_t size = 128 << g->n;
    uint32_t base = im.data_off + ((uint32_t)track * g->sides + side) * g->sectors * size;
    for (int i = 0; i < g->sectors && i < IMG_MAX_SECT; i++) {
      if ((size_t)(i + 1) * size > bufsize)
        break;
      img_sect_t &s = L.s[L.count++];
      s.c = track;
      s.h = side;
      s.r = im.r_2hs ? (i == 0 && track == 0 ? 1 : i + 10) : i + 1;
      s.n = g->n;
      s.deleted = 0;
      s.size = size;
      s.buf_off = i * size;
      s.file_off = base + i * size;
      s.idam_pos = 0;
    }
    size_t total = (size_t)L.count * size;
    if (!f.seek(base) || f.read(buf, total) != (int)total) {
      IMG_LOG("RAW: read error track %d side %d\n", track, side);
      L.count = 0;
      return false;
    }
    return true;
  }
  if (im.type == IMG_D88) {
    uint32_t t = (uint32_t)track * 2 + side;
    uint32_t tab = 0x20 + 4 * t;
    if (t >= D88_MAX_TRACKS || tab + 4 > im.d88_hdr_end)
      return true;
    uint8_t o[4];
    if (!f.seek(tab) || f.read(o, 4) != 4)
      return false;
    uint32_t pos = img_le32(o);
    if (!pos)
      return true; // unformatted track
    uint16_t nsec = 0, used = 0;
    for (int i = 0; i < IMG_MAX_SECT; i++) {
      uint8_t h[16];
      if (!f.seek(pos) || f.read(h, 16) != 16)
        break;
      if (i == 0)
        nsec = img_le16(h + 4);
      if (i >= nsec)
        break;
      uint16_t size = img_le16(h + 14);
      if ((size_t)used + size > bufsize) {
        IMG_LOG("D88: track %d side %d too big, %d sectors used\n", track, side, i);
        break;
      }
      if (size && f.read(buf + used, size) != (int)size)
        break;
      img_sect_t &s = L.s[L.count++];
      s.c = h[0];
      s.h = h[1];
      s.r = h[2];
      s.n = h[3];
      s.deleted = h[7] != 0;
      s.size = size;
      s.buf_off = used;
      s.file_off = pos + 16;
      s.idam_pos = 0;
      if (h[6] == 0x40)
        IMG_LOG("D88: FM sector on track %d side %d encoded as MFM\n", track, side);
      used += size;
      pos += 16 + size;
    }
    return true;
  }
  return false;
}

// Read again the data of a side already loaded (the list is kept)
template <class F> bool img_reload_data(img_t &im, F &f, int side, uint8_t *buf) {
  img_side_t &L = im.side[side];
  for (int i = 0; i < L.count; i++) {
    const img_sect_t &s = L.s[i];
    if (!f.seek(s.file_off) || f.read(buf + s.buf_off, s.size) != (int)s.size)
      return false;
  }
  return true;
}

// Save the sectors in mask (bit i = L.s[i]) back into the image
template <class F>
bool img_write_sectors(img_t &im, F &f, int side, const uint8_t *buf, uint32_t mask) {
  img_side_t &L = im.side[side];
  bool ok = true;
  for (int i = 0; i < L.count; i++) {
    if (!(mask & (1u << i)))
      continue;
    const img_sect_t &s = L.s[i];
    ok = ok && f.seek(s.file_off) && f.write(buf + s.buf_off, s.size) == s.size;
  }
  return ok && f.sync();
}

// Generate the MFM flux (compact form, 1 bit per cell) of a side.
// Gap 3 is reduced if needed so that all the sectors fit in one revolution.
static void img_encode_side(img_side_t &L, const uint8_t *buf, uint8_t *pulses,
                            size_t n_bytes) {
  const mfm_io_settings_t *st = &standard_mfm;
  mfm_io_t io = {};
  io.encode_compact = true;
  io.pulses = pulses;
  io.n_pulses = n_bytes;
  io.settings = st;
  io.flux_byte = mfm_io_flux_byte_compact;
  io.encode_raw = mfm_io_encode_raw_mfm;

  // data bytes available in one revolution (each byte = 16 cells = 2 bytes)
  int avail = (int)(n_bytes / 2);
  int used = st->gap_4a + st->gap_presync + 3 + 1 + st->gap_1 + st->gap_presync + 3;
  for (int i = 0; i < L.count; i++)
    used += 7 + st->gap_2 + st->gap_presync + 3 + 1 + L.s[i].size + 2 +
            st->gap_presync + 3;
  int gap3_fit = L.count ? (avail - used - 16) / L.count : 0;
  if (L.count && gap3_fit < 1) {
    IMG_LOG("Track %d does not fit in one revolution\n", L.track);
    gap3_fit = 1;
  }

  mfm_io_encode_iam(&io);
  mfm_io_encode_gap_and_sync(&io, st->gap_1);
  for (int i = 0; i < L.count; i++) {
    img_sect_t &s = L.s[i];
    s.idam_pos = io.pos;
    uint8_t id[5] = {MFM_IO_IDAM, s.c, s.h, s.r, s.n};
    mfm_io_crc_preload(&io);
    mfm_io_encode_buf_crc(&io, id, sizeof(id));
    mfm_io_encode_crc(&io);
    mfm_io_encode_gap_and_sync(&io, st->gap_2);
    mfm_io_crc_preload(&io);
    mfm_io_encode_byte_crc(&io, s.deleted ? 0xf8 : MFM_IO_DAM);
    mfm_io_encode_buf_crc(&io, buf + s.buf_off, s.size);
    mfm_io_encode_crc(&io);
    int gap3 = st->gap_3[s.n & 7];
    if (gap3 > gap3_fit)
      gap3 = gap3_fit;
    mfm_io_encode_gap_and_sync(&io, gap3);
  }
  while (!mfm_io_eof(&io))
    mfm_io_encode_byte(&io, st->gap_byte);
}

// Sector under the head at byte offset raw_pos of the compact flux: the last
// one whose ID mark has already passed (a write follows its ID field)
static int img_sector_at(const img_side_t &L, int raw_pos) {
  int best = -1;
  for (int i = 0; i < L.count; i++)
    if (L.s[i].idam_pos <= raw_pos)
      best = i;
  return best;
}

static int img_find_sector(const img_side_t &L, uint8_t r, uint8_t n) {
  for (int i = 0; i < L.count; i++)
    if (L.s[i].r == r && L.s[i].n == n)
      return i;
  return -1;
}

// Decode one write (pulse lengths in counts, t1 = counts per bit cell).
// raw_pos: byte offset in the read flux where the write started, -1 unknown.
// overflow: the capture ended before the write did (long track format).
// Valid sectors are copied into buf; returns the bitmap of sectors updated.
static uint32_t img_decode_write(uint8_t *pulses, size_t n_pulses, bool overflow,
                                 int raw_pos, const img_side_t &L, uint8_t *buf,
                                 uint8_t *tmp, size_t tmp_size, float t1) {
  mfm_io_t io = {};
  io.T1_nom = lroundf(t1);
  io.T2_max = lroundf(t1 * 2.5f);
  io.T3_max = lroundf(t1 * 3.5f);
  io.pulses = pulses;
  io.n_pulses = n_pulses;
  io.settings = &standard_mfm;

  int pos_sector = raw_pos >= 0 ? img_sector_at(L, raw_pos) : -1;
  int id_sector = -1; // sector of the ID field just read (track format)
  bool saw_idam = false;
  uint32_t mask = 0;
  int fill = -1;      // fill byte of a formatted track, -2 = not uniform

  while (!mfm_io_eof(&io)) {
    if (!skip_triple_sync_mark(&io))
      break;
    // Whole decoder state, to read the same field again as data
    const mfm_io_t saved = io;
    uint8_t mark, idam[mfm_io_idam_size], crc_buf[mfm_io_crc_size];

    // ID field? (present only when a track is formatted)
    uint16_t crc = receive_crc(&io, &mark, 1, idam, sizeof(idam), crc_buf,
                               sizeof(crc_buf), NULL);
    if (mark == MFM_IO_IDAM) {
      saw_idam = true;
      id_sector = crc == 0 ? img_find_sector(L, idam[2], idam[3]) : -1;
      if (crc == 0 && id_sector < 0)
        IMG_LOG("WR: format with sector R=%d N=%d not in the image, ignored\n",
                idam[2], idam[3]);
      continue;
    }

    io = saved;
    int r = saw_idam ? id_sector : pos_sector;
    size_t len = r >= 0 ? (size_t)128 << (L.s[r].n & 7) : 128;
    if (len > tmp_size) {
      IMG_LOG("WR: sector size %u not supported\n", (unsigned)len);
      break;
    }
    crc = receive_crc(&io, &mark, 1, tmp, len, crc_buf, sizeof(crc_buf), NULL);
    if (mark != MFM_IO_DAM && mark != 0xf8 /* deleted data */) {
      id_sector = -1;
      continue;
    }
    id_sector = -1;
    pos_sector = -1; // one data field per write position
    if ((crc != 0 || r < 0) && overflow && mfm_io_eof(&io))
      break; // data field cut by the end of the capture buffer
    if (crc != 0 || r < 0) {
      IMG_LOG("WR: bad data field (crc=%04x sector=%d)\n", crc, r >= 0 ? L.s[r].r : -1);
      continue;
    }
    const img_sect_t &s = L.s[r];
    memcpy(buf + s.buf_off, tmp, len < s.size ? len : s.size);
    mask |= 1u << r;
    if (saw_idam && fill != -2) {
      bool uniform = true;
      for (size_t i = 1; i < len && uniform; i++)
        uniform = tmp[i] == tmp[0];
      fill = uniform ? tmp[0] : -2;
    }
  }

  // Track format longer than the capture buffer: the sectors that did not
  // fit are written with the same fill byte as the ones decoded.
  if (saw_idam && overflow && fill >= 0) {
    int decoded = __builtin_popcount(mask);
    IMG_LOG("WR: format: %d sectors decoded, %d filled with %02X\n", decoded,
            L.count - decoded, fill);
    for (int i = 0; i < L.count; i++) {
      if (!(mask & (1u << i))) {
        memset(buf + L.s[i].buf_off, fill, L.s[i].size);
        mask |= 1u << i;
      }
    }
  }
  return mask;
}
