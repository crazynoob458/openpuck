from pathlib import Path
import subprocess, tempfile
root = Path(__file__).resolve().parents[2]
s = (root/'OpenPuck/config.cpp').read_text()
# the per-type defaults and rumbleStoreActive(), as compiled into config.cpp
slice_ = s[s.index('uint8_t g_typeLimitKnee['):s.index('void applyActiveType()')]
test = r'''#include <cassert>
#include <cstdint>
#define ET_COUNT 4
#define ET_XBOX 0
#define ET_SWITCH 1
#define ET_DS4 2
#define ET_DS5 3
#define HAPTIC_LIMIT_KNEE_DEFAULT 70u
#define RUMBLE_SCALE_PCT 200u
#define AUDIO_STYLE_RUMBLE 0
#define AUDIO_STYLE_TONE 1
#define AUDIO_STYLE_SPLIT 2
#define AUDIO_STYLE_WAVE 3
uint8_t g_etype = 0;
uint16_t g_rumbleScale = RUMBLE_SCALE_PCT;
''' + slice_ + r'''
int main()
{
	// the Switch starts on tones at 100% grip strength; every other type on wave at 200%
	for (int et = 0; et < ET_COUNT; et++) {
		assert(g_typeHapticStyle[et] == (et == ET_SWITCH ? AUDIO_STYLE_TONE : AUDIO_STYLE_WAVE));
		assert(g_typeRumbleScale[et] == (et == ET_SWITCH ? 100u : 200u));
		assert(g_typeRumbleScaleDefault[et] == g_typeRumbleScale[et]);
		assert(g_typeLimitKnee[et] == 70u);
	}
	// an edit to the live strength lands on the active type only
	g_etype = ET_SWITCH;
	g_rumbleScale = 300;
	rumbleStoreActive();
	assert(g_typeRumbleScale[ET_SWITCH] == 300u && g_typeRumbleScale[ET_DS5] == 200u);
	// a puck mode (no emulated type) stores nothing
	g_etype = ET_COUNT;
	g_rumbleScale = 50;
	rumbleStoreActive();
	for (int et = 0; et < ET_COUNT; et++)
		assert(g_typeRumbleScale[et] != 50u);
	return 0;
}
'''
with tempfile.TemporaryDirectory() as td:
	p = Path(td)/'test.cpp'
	p.write_text(test)
	subprocess.run(['g++', '-std=c++11', '-g', '-Wall', '-Wextra', '-Werror', '-fsanitize=address,undefined', str(p), '-o', td+'/test'], check=True)
	subprocess.run([td+'/test'], check=True, env={'ASAN_OPTIONS': 'detect_leaks=0'})
print('per-type haptic defaults tests passed')
