// Sanity checks on the characterization matrix itself. The keymap tests rely
// on it being unambiguous and well formed.
#include "keymap_expectations.h"
#include "test_runner.h"

using namespace keymap_expect;

namespace {

bool isBankKey(char key) { return key != '\0' && strchr(kBankKeys, key) != nullptr; }

bool isSentinel(const char *action) {
  return strcmp(action, "unassigned") == 0 || strcmp(action, "none") == 0 ||
         strcmp(action, "-") == 0;
}

}  // namespace

TEST(expectations_rows_are_well_formed) {
  for (const Row &r : kRows) {
    CHECK(r.families != 0);
    CHECK_EQ(r.families & ~ALL, 0);
    CHECK(r.bank >= 1 && r.bank <= 9);
    CHECK(r.bank != 7);
    CHECK(isBankKey(r.key));
    CHECK(r.shortAction && *r.shortAction);
    CHECK(r.longAction && *r.longAction);
    CHECK(r.doubleAction && *r.doubleAction);
    // Each sentinel is only valid in its own column.
    CHECK(!isSentinel(r.shortAction) || strcmp(r.shortAction, "unassigned") == 0);
    CHECK(!isSentinel(r.longAction) || strcmp(r.longAction, "none") == 0);
    CHECK(strcmp(r.doubleAction, "unassigned") != 0);
  }
}

TEST(expectations_cover_each_key_at_most_once) {
  for (const Family &f : kFamilies) {
    for (uint8_t bank = 1; bank <= 9; ++bank) {
      for (const char *k = kBankKeys; *k; ++k) {
        int matches = 0;
        for (const Row &r : kRows) {
          if ((r.families & f.bit) && r.bank == bank && r.key == *k) ++matches;
        }
        if (matches > 1) {
          fprintf(stderr, "  %s bank %u key %c: %d rows\n", f.name, bank, *k, matches);
        }
        CHECK(matches <= 1);
      }
    }
  }
}

// A key with an assignment for one family has an explicit row for every family.
TEST(expectations_assigned_keys_cover_all_families) {
  for (const Row &r : kRows) {
    uint16_t covered = 0;
    for (const Row &other : kRows) {
      if (other.bank == r.bank && other.key == r.key) covered |= other.families;
    }
    if (covered != ALL) {
      fprintf(stderr, "  bank %u key %c: families 0x%x not covered\n", r.bank, r.key,
              ALL & ~covered);
    }
    CHECK_EQ(covered, ALL);
  }
}

TEST(expectations_lookup_defaults_to_unassigned) {
  const Row &r = lookup(IC7300, 7, '1');
  CHECK_EQ(r.shortAction, "unassigned");
  CHECK_EQ(r.longAction, "none");
  CHECK_EQ(r.doubleAction, "-");
  CHECK_EQ(lookup(FT817, 1, 'A').shortAction, "unassigned");
}

TEST(expectations_lookup_picks_the_family_row) {
  CHECK_EQ(lookup(FTDX10, 2, '4').shortAction, "send(GT?)");
  CHECK_EQ(lookup(IC706, 2, '4').shortAction, "queryBank2NrLevel");
  CHECK_EQ(lookup(TS480, 2, '4').shortAction, "unassigned");
  CHECK_EQ(lookup(LIGHT, 9, '4').shortAction, "selectBank9DirectProfile(4)");
  CHECK_EQ(lookup(IC7300, 9, '4').shortAction, "queryBank9TuningSpeech");
}
