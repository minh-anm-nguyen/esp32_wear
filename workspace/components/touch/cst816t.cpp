#include "cst816t.hpp"

namespace touch {

// ------------------------------------------------------------------- strings

const char* toString(FrameStatus s)
{
    switch (s) {
    case FrameStatus::Valid:          return "Valid";
    case FrameStatus::NoFinger:       return "NoFinger";
    case FrameStatus::AllOnes:        return "AllOnes";
    case FrameStatus::BadFingerCount: return "BadFingerCount";
    case FrameStatus::ReservedEvent:  return "ReservedEvent";
    case FrameStatus::OutOfRange:     return "OutOfRange";
    }
    return "?";
}

const char* toString(EventFlag e)
{
    switch (e) {
    case EventFlag::Down:     return "Down";
    case EventFlag::Up:       return "Up";
    case EventFlag::Contact:  return "Contact";
    case EventFlag::Reserved: return "Reserved";
    }
    return "?";
}

const char* gestureName(uint8_t code)
{
    switch (code) {
    case reg::GESTURE_NONE:         return "none";
    case reg::GESTURE_SLIDE_UP:     return "slide-up";
    case reg::GESTURE_SLIDE_DOWN:   return "slide-down";
    case reg::GESTURE_SLIDE_LEFT:   return "slide-left";
    case reg::GESTURE_SLIDE_RIGHT:  return "slide-right";
    case reg::GESTURE_SINGLE_CLICK: return "click";
    case reg::GESTURE_DOUBLE_CLICK: return "double-click";
    case reg::GESTURE_LONG_PRESS:   return "long-press";
    case reg::GESTURE_BIG_PALM:     return "big-palm";
    default:                        return "unknown";
    }
}

namespace {

bool knownGesture(uint8_t code)
{
    switch (code) {
    case reg::GESTURE_NONE:
    case reg::GESTURE_SLIDE_UP:
    case reg::GESTURE_SLIDE_DOWN:
    case reg::GESTURE_SLIDE_LEFT:
    case reg::GESTURE_SLIDE_RIGHT:
    case reg::GESTURE_SINGLE_CLICK:
    case reg::GESTURE_DOUBLE_CLICK:
    case reg::GESTURE_LONG_PRESS:
    case reg::GESTURE_BIG_PALM:
        return true;
    default:
        return false;
    }
}

// Returns true if the coordinate is usable, pulling back a value that sits
// exactly within edgeTolerance of the last pixel. Anything further out is a
// corrupt frame -- clamping THAT would turn garbage into a press on whatever
// widget happens to sit at the screen edge.
bool fitCoordinate(uint16_t& value, uint16_t limit, uint16_t tolerance, bool& clamped)
{
    if (limit == 0) {
        return false;
    }
    const uint16_t last = static_cast<uint16_t>(limit - 1);
    if (value <= last) {
        return true;
    }
    if (value <= static_cast<uint16_t>(last + tolerance)) {
        value   = last;
        clamped = true;
        return true;
    }
    return false;
}

}  // namespace

// --------------------------------------------------------------------- parse

ParsedFrame parseFrame(const uint8_t* frame, std::size_t len, const RawLimits& limits)
{
    ParsedFrame out{};

    if (frame == nullptr || len < reg::FRAME_LEN) {
        out.status = FrameStatus::AllOnes;  // nothing usable arrived
        return out;
    }

    // Checked FIRST. An all-0xFF frame is what a floating bus reads back, and
    // every field below would otherwise decode it into a plausible-looking
    // touch: finger count 255, event 11, coordinates 0xFFF.
    bool allOnes = true;
    for (std::size_t i = 0; i < reg::FRAME_LEN; ++i) {
        if (frame[i] != 0xFF) {
            allOnes = false;
            break;
        }
    }
    if (allOnes) {
        out.status = FrameStatus::AllOnes;
        return out;
    }

    const uint8_t gesture = frame[reg::GESTURE_ID - reg::FRAME_BASE];
    const uint8_t fingers = frame[reg::FINGER_NUM - reg::FRAME_BASE];
    const uint8_t xh      = frame[reg::XPOS_H - reg::FRAME_BASE];
    const uint8_t xl      = frame[reg::XPOS_L - reg::FRAME_BASE];
    const uint8_t yh      = frame[reg::YPOS_H - reg::FRAME_BASE];
    const uint8_t yl      = frame[reg::YPOS_L - reg::FRAME_BASE];

    out.gestureCode    = gesture;
    out.gestureUnknown = !knownGesture(gesture);
    out.fingers        = fingers;

    // The event flag is read BEFORE the coordinate is masked out of the same
    // byte. Masking first destroys it, and the bug is invisible: coordinates
    // stay correct and only press/release goes wrong.
    out.event = static_cast<EventFlag>((xh & reg::EVENT_MASK) >> reg::EVENT_SHIFT);

    out.rawX = static_cast<uint16_t>((static_cast<uint16_t>(xh & reg::COORD_H_MASK) << 8) | xl);
    out.rawY = static_cast<uint16_t>((static_cast<uint16_t>(yh & reg::COORD_H_MASK) << 8) | yl);

    // Recorded even when the frame is otherwise fine. Which of the two to
    // believe on release is a hardware question, and the counter is how it
    // gets answered from a log instead of from an opinion.
    out.fingerEventMismatch = (fingers == 0 && out.event == EventFlag::Down)
                           || (fingers == 1 && out.event == EventFlag::Up);

    if (out.event == EventFlag::Reserved) {
        out.status = FrameStatus::ReservedEvent;
        return out;
    }

    if (fingers == 0) {
        out.status = FrameStatus::NoFinger;
        return out;
    }

    if (fingers > 1) {
        // Single-touch part. A count above 1 means the frame was not decoded
        // the way this driver believes, so nothing else in it is trustworthy.
        out.status = FrameStatus::BadFingerCount;
        return out;
    }

    bool clamped = false;
    if (!fitCoordinate(out.rawX, limits.width, limits.edgeTolerance, clamped)
        || !fitCoordinate(out.rawY, limits.height, limits.edgeTolerance, clamped)) {
        out.status = FrameStatus::OutOfRange;
        return out;
    }
    out.edgeClamped = clamped;
    out.status      = FrameStatus::Valid;
    return out;
}

// ------------------------------------------------------------ config encoding

uint8_t Cst816t::irqCtlByte(const ChipConfig& config)
{
    uint8_t v = 0;
    if (config.irqSelfTest) v |= reg::IRQ_EN_TEST;
    if (config.irqOnTouch)  v |= reg::IRQ_EN_TOUCH;
    if (config.irqOnChange) v |= reg::IRQ_EN_CHANGE;
    if (config.irqOnMotion) v |= reg::IRQ_EN_MOTION;
    return v;
}

uint8_t Cst816t::motionMaskByte(const ChipConfig& config)
{
    return config.enableDoubleClick ? reg::MOTION_EN_DCLICK : 0;
}

uint8_t Cst816t::errResetByte(const ChipConfig& config)
{
    uint8_t v = 0;
    if (config.resetOnLongPress) v |= reg::ERR_RESET_LONGPRESS;
    if (config.resetOnLargeArea) v |= reg::ERR_RESET_LARGEAREA;
    if (config.resetOnTwoFinger) v |= reg::ERR_RESET_TWOFINGER;
    return v;
}

uint8_t Cst816t::autoSleepByte(const ChipConfig& config)
{
    return config.disableAutoSleep ? reg::AUTO_SLEEP_DISABLED : reg::AUTO_SLEEP_ENABLED;
}

uint8_t Cst816t::irqPulseByte(const ChipConfig& config)
{
    // Clamped rather than trusted. The T document gives 1..5 ms; a value from
    // an S-derived example would be 10, which is outside that and would be
    // silently ignored or misinterpreted.
    uint8_t v = config.irqPulseMs;
    if (v < reg::IRQ_PULSE_MIN_MS) v = reg::IRQ_PULSE_MIN_MS;
    if (v > reg::IRQ_PULSE_MAX_MS) v = reg::IRQ_PULSE_MAX_MS;
    return v;
}

// ------------------------------------------------------------------ identify

ChipInfo Cst816t::identify()
{
    ChipInfo info{};
    uint8_t  buf[reg::IDENT_LEN]{};

    info.read = bus_.readRegs(reg::IDENT_BASE, buf, sizeof(buf));
    if (!info.read) {
        return info;
    }

    info.chipId    = buf[0];
    info.projId    = buf[1];
    info.fwVersion = buf[2];
    info.factoryId = buf[3];
    return info;
}

// --------------------------------------------------------------- applyConfig

bool Cst816t::writeVerified(ConfigResult& out, uint8_t regAddr, uint8_t value, bool verify)
{
    if (out.count >= ConfigResult::kMaxEntries) {
        return false;
    }

    ConfigResult::Entry& e = out.entries[out.count++];
    e.regAddr = regAddr;
    e.wrote = value;

    e.writeOk = bus_.writeReg(regAddr, value);
    if (!e.writeOk) {
        return false;
    }

    if (!verify) {
        // Not "verified" -- nothing was checked. Saying otherwise would make
        // allVerified() lie about a configuration nobody looked at.
        return true;
    }

    uint8_t back = 0;
    if (!bus_.readRegs(regAddr, &back, 1)) {
        return false;
    }
    e.readBack = back;
    e.verified = (back == value);
    return true;
}

ConfigResult Cst816t::applyConfig(const ChipConfig& config)
{
    ConfigResult out{};

    // ORDER IS THE POINT.
    //
    // IRQ_CTL is written LAST, so the chip is not pulsing the interrupt line
    // while the rest of its behaviour is still half-applied. Everything before
    // it is inert until then.
    //
    // A failure does not abort the sequence: the remaining registers are still
    // attempted so the log shows WHICH ones the chip refused, not just the
    // first. On a chip that has slipped into standby every one of them NACKs,
    // and that pattern is itself the diagnosis.
    writeVerified(out, reg::DIS_AUTO_SLEEP, autoSleepByte(config), config.verifyWrites);
    writeVerified(out, reg::IRQ_PULSE_WIDTH, irqPulseByte(config), config.verifyWrites);
    writeVerified(out, reg::MOTION_MASK, motionMaskByte(config), config.verifyWrites);
    writeVerified(out, reg::ERR_RESET_CTL, errResetByte(config), config.verifyWrites);
    writeVerified(out, reg::IRQ_CTL, irqCtlByte(config), config.verifyWrites);

    return out;
}

// ----------------------------------------------------------------- readFrame

ParsedFrame Cst816t::readFrame(const RawLimits& limits, bool& busOk)
{
    return readFrame(limits, busOk, nullptr);
}

ParsedFrame Cst816t::readFrame(const RawLimits& limits, bool& busOk, uint8_t* rawOut)
{
    uint8_t buf[reg::FRAME_LEN]{};

    // ONE transaction with a repeated START -- i2c::Bus::readRegs guarantees
    // that. Two transactions would let the finger move between X and Y, and
    // would let another task take the shared bus in the middle of a frame.
    busOk = bus_.readRegs(reg::FRAME_BASE, buf, sizeof(buf));

    if (rawOut != nullptr) {
        for (std::size_t i = 0; i < sizeof(buf); ++i) {
            rawOut[i] = buf[i];
        }
    }

    if (!busOk) {
        ParsedFrame out{};
        out.status = FrameStatus::NoFinger;  // caller must branch on busOk, not on this
        return out;
    }
    return parseFrame(buf, sizeof(buf), limits);
}

// ------------------------------------------------------------ enterDeepSleep

bool Cst816t::enterDeepSleep()
{
    return bus_.writeReg(reg::SLEEP_MODE, reg::SLEEP_MODE_ENTER);
}

}  // namespace touch
