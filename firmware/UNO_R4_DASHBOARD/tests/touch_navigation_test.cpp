#include "touch_navigation.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>

using namespace TouchNavigation;

TouchSample sample(Phase phase, int x, int y = 160, Source source = Source::VIRTUAL,
                   uint8_t finger = 0, uint8_t contacts = 1)
{
  return {phase, source, int16_t(x), int16_t(y), finger, contacts};
}

void navigation(Source source)
{
  Navigator nav;
  nav.setEnabled(true);
  nav.accept(sample(Phase::UP, 0, 0, Source::PHYSICAL), 0);
  uint32_t now = 200;
  auto swipe = [&](int start, int end, int expected) {
    const int before = nav.page();
    nav.accept(sample(Phase::DOWN, start, 160, source), now);
    nav.accept(sample(Phase::MOVE, end, 160, source), now + 20);
    assert(nav.page() == before);
    nav.accept(sample(Phase::UP, end, 160, source), now + 40);
    assert(nav.page() == expected);
    now += 200;
  };
  swipe(100, 300, -1); // Dashboard -> L
  swipe(100, 300, -1); // Non-wrap L boundary
  swipe(300, 100, 0);  // L -> Dashboard
  swipe(300, 100, 1);  // Dashboard -> R
  swipe(300, 100, 1);  // Non-wrap R boundary
  swipe(100, 300, 0);  // R -> Dashboard
  swipe(100, 159, 0);
  swipe(100, 160, -1); // Inclusive 60 px threshold
}

int main()
{
  navigation(Source::PHYSICAL);
  navigation(Source::VIRTUAL);
  Gesture gesture;
  gesture.accept(sample(Phase::DOWN, 100, 50));
  assert(gesture.accept(sample(Phase::UP, 170, 200)) == 0); // Vertical dominant
  gesture.accept(sample(Phase::DOWN, 100));
  assert(gesture.accept(sample(Phase::UP, 160, 220)) == 0); // Equal dx/dy
  gesture.accept(sample(Phase::DOWN, 100));
  assert(gesture.accept(sample(Phase::UP, 100)) == 0); // Tap
  for (const auto &invalid : {
      sample(Phase::MOVE, 200, 160, Source::VIRTUAL, 0, 2),
      sample(Phase::MOVE, 480), sample(Phase::MOVE, -1),
      sample(Phase::MOVE, 200, 320), sample(Phase::MOVE, 200, -1),
      sample(Phase::MOVE, 200, 160, Source::VIRTUAL, 1),
      sample(Phase::MOVE, 200, 160, Source::PHYSICAL),
      sample(Phase::CANCEL, 200)})
  {
    gesture.accept(sample(Phase::DOWN, 100));
    assert(gesture.accept(invalid) == 0);
    assert(gesture.accept(sample(Phase::UP, 300)) == 0);
  }
  // Fresh physical polls never terminate a virtual gesture. Physical DOWN
  // cancels it, owns the recognizer through release, then enforces the guard.
  Navigator nav;
  nav.setEnabled(true);
  nav.accept(sample(Phase::UP, 0, 0, Source::PHYSICAL), 0);
  nav.accept(sample(Phase::DOWN, 100), 200);
  nav.accept(sample(Phase::UP, 0, 0, Source::PHYSICAL), 210);
  assert(nav.accept(sample(Phase::UP, 300), 220));
  nav.setEnabled(true);
  nav.accept(sample(Phase::DOWN, 100), 300);
  nav.accept(sample(Phase::DOWN, 300, 160, Source::PHYSICAL), 310);
  nav.accept(sample(Phase::DOWN, 100), 320);
  nav.accept(sample(Phase::UP, 300), 330);
  assert(nav.page() == 0);
  assert(nav.accept(sample(Phase::UP, 100, 160, Source::PHYSICAL), 340));
  assert(nav.page() == 1);
  nav.accept(sample(Phase::DOWN, 100), 489);
  nav.accept(sample(Phase::UP, 300), 491);
  assert(nav.page() == 1); // Rejected DOWN cannot become a swipe after the guard.
  nav.accept(sample(Phase::DOWN, 100), 500);
  nav.accept(sample(Phase::CANCEL, 300), 510);
  assert(!nav.accept(sample(Phase::UP, 300), 520));
  nav.accept(sample(Phase::DOWN, 100), 600);
  assert(!nav.accept(sample(Phase::UP, 300), 1600)); // Timeout at release
  nav.accept(sample(Phase::DOWN, 100), 1700);
  nav.cancelVirtual();
  assert(!nav.accept(sample(Phase::UP, 300), 1710));
  nav.setEnabled(false);
  nav.accept(sample(Phase::DOWN, 100), 1800);
  assert(!nav.accept(sample(Phase::UP, 300), 1810));

  TouchSample parsed{};
  for (const char *line : {"VT D 0 0", "VT M 479 319", "VT U 160 20", "VT C"})
    assert(parseVirtualTouch(line, parsed));
  for (const char *line : {"", "V", "VT", "VT ", "VT D", "VT D ",
      "VT D 0", "VT D 0 ", "VT D 480 0", "VT D 0 320", "VT D -1 2",
      "VT M 1.5 2", "VT U 1 2 trailing", "VT X 1 2", "VT C 1 2",
      "VT D 999999999999 2", "VT D +1 2", "VT D 1 2\nVT U 2 3"})
    assert(!parseVirtualTouch(line, parsed));
  std::puts("touch navigation: PASS (physical + virtual, arbitration, cancellation, VT grammar)");
}
