// Runs every registered TEST. An optional argument runs only the tests whose
// name contains it. Exit status is non-zero if any test failed.
#include "test_runner.h"

int main(int argc, char **argv) {
  const char *filter = argc > 1 ? argv[1] : nullptr;
  int run = 0;
  int failed = 0;

  for (test::Case *c = test::g_head; c; c = c->next) {
    if (filter && !strstr(c->name, filter)) {
      continue;
    }
    test::g_checkFailures = 0;
    c->fn();
    ++run;
    if (test::g_checkFailures > 0) {
      ++failed;
      fprintf(stderr, "FAIL %s\n", c->name);
    }
  }

  if (run == 0) {
    fprintf(stderr, "No tests matched\n");
    return 1;
  }
  printf("%d/%d tests passed\n", run - failed, run);
  return failed > 0 ? 1 : 0;
}
