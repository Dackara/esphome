import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import binary_sensor, button, light, output, switch
from esphome.const import CONF_ID, CONF_PIN, CONF_OUTPUT_ID

CODEOWNERS = ["@Dackara"]
MULTI_CONF = True
DEPENDENCIES = []
AUTO_LOAD = ["binary_sensor", "button", "light", "output", "switch"]

teleruptor_ns = cg.esphome_ns.namespace("teleruptor")
Teleruptor = teleruptor_ns.class_("Teleruptor", cg.Component)
TeleruptorLightOutput = teleruptor_ns.class_("TeleruptorLightOutput", light.LightOutput)
TeleruptorSwitch = teleruptor_ns.class_("TeleruptorSwitch", switch.Switch, cg.Component)
TeleruptorButton = teleruptor_ns.class_("TeleruptorButton", button.Button, cg.Component)
TeleruptorBinarySensor = teleruptor_ns.class_(
    "TeleruptorBinarySensor", binary_sensor.BinarySensor, cg.Component
)

CONF_MODE = "mode"
CONF_DETECTION = "detection"
CONF_INPUT = "input"
CONF_DEBOUNCE = "debounce"
CONF_PULSE_OUTPUT = "pulse_output"
CONF_RELAY_OUTPUT = "relay_output"
CONF_PULSE_DURATION = "pulse_duration"
CONF_COMMAND_COOLDOWN = "command_cooldown"
CONF_FEEDBACK_TIMEOUT = "feedback_timeout"
CONF_STATE_INVERTED = "state_inverted"
CONF_FEEDBACK_INVERTED = "feedback_inverted"
CONF_SYNC_ON_BOOT = "sync_on_boot"
CONF_DEFAULT_STATE = "default_state"
CONF_TELERUPTOR_ID = "teleruptor_id"
CONF_ALLOW_PULSE_WITHOUT_FEEDBACK = "allow_pulse_without_feedback"
CONF_BLOCK_AFTER_FEEDBACK_TIMEOUT = "block_after_feedback_timeout"
CONF_UNSTABLE_DETECTION = "unstable_detection"
CONF_ENABLED = "enabled"
CONF_WINDOW = "window"
CONF_MAX_CHANGES = "max_changes"
CONF_BLOCK_COMMANDS = "block_commands"

CONF_LIGHT = "light"
CONF_SWITCH = "switch"
CONF_BUTTON = "button"
CONF_BINARY_SENSOR = "binary_sensor"

MODE_PULSE_FEEDBACK = "pulse_feedback"
MODE_PULSE_OPTIMISTIC = "pulse_optimistic"
MODE_RELAY_LATCH = "relay_latch"

MODE_TO_INT = {
    MODE_PULSE_FEEDBACK: 0,
    MODE_PULSE_OPTIMISTIC: 1,
    MODE_RELAY_LATCH: 2,
}

BINARY_INPUT_SCHEMA = cv.Any(
    cv.use_id(binary_sensor.BinarySensor),
    cv.Schema(
        {
            cv.Required(CONF_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_DEBOUNCE, default="100ms"): cv.positive_time_period_milliseconds,
        }
    ),
)

BUTTON_INPUT_SCHEMA = cv.Any(
    cv.use_id(binary_sensor.BinarySensor),
    cv.Schema(
        {
            cv.Required(CONF_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_DEBOUNCE, default="50ms"): cv.positive_time_period_milliseconds,
        }
    ),
)

BINARY_OUTPUT_SCHEMA = cv.Any(
    cv.use_id(output.BinaryOutput),
    cv.Schema(
        {
            cv.Required(CONF_PIN): pins.gpio_output_pin_schema,
        }
    ),
)

UNSTABLE_DETECTION_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ENABLED, default=True): cv.boolean,
        cv.Optional(CONF_WINDOW, default="10s"): cv.positive_time_period_milliseconds,
        cv.Optional(CONF_MAX_CHANGES, default=10): cv.int_range(min=1),
        cv.Optional(CONF_BLOCK_COMMANDS, default=False): cv.boolean,
    }
)

TELERUPTOR_LIGHT_SCHEMA = light.BINARY_LIGHT_SCHEMA.extend(
    {
        cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(TeleruptorLightOutput),
    }
)
TELERUPTOR_SWITCH_SCHEMA = switch.switch_schema(TeleruptorSwitch).extend(cv.COMPONENT_SCHEMA)
TELERUPTOR_BUTTON_SCHEMA = button.button_schema(TeleruptorButton).extend(cv.COMPONENT_SCHEMA)
TELERUPTOR_BINARY_SENSOR_SCHEMA = binary_sensor.binary_sensor_schema(TeleruptorBinarySensor).extend(
    cv.COMPONENT_SCHEMA
)


def _migrate_feedback_inverted(config):
    if CONF_FEEDBACK_INVERTED in config:
        if CONF_STATE_INVERTED in config:
            raise cv.Invalid(
                f"Use either {CONF_STATE_INVERTED} or legacy {CONF_FEEDBACK_INVERTED}, not both"
            )
        config[CONF_STATE_INVERTED] = config.pop(CONF_FEEDBACK_INVERTED)
    return config


def _validate_mode_config(config):
    mode = config[CONF_MODE]

    if mode == MODE_PULSE_FEEDBACK:
        if CONF_DETECTION not in config:
            raise cv.Invalid(f"{CONF_DETECTION} is required when {CONF_MODE}: {MODE_PULSE_FEEDBACK}")
        if CONF_PULSE_OUTPUT not in config:
            raise cv.Invalid(f"{CONF_PULSE_OUTPUT} is required when {CONF_MODE}: {MODE_PULSE_FEEDBACK}")
        if CONF_INPUT in config:
            raise cv.Invalid(f"{CONF_INPUT} is only valid when {CONF_MODE}: {MODE_RELAY_LATCH}")
        if CONF_RELAY_OUTPUT in config:
            raise cv.Invalid(f"{CONF_RELAY_OUTPUT} is only valid when {CONF_MODE}: {MODE_RELAY_LATCH}")

    elif mode == MODE_PULSE_OPTIMISTIC:
        if CONF_PULSE_OUTPUT not in config:
            raise cv.Invalid(f"{CONF_PULSE_OUTPUT} is required when {CONF_MODE}: {MODE_PULSE_OPTIMISTIC}")
        if CONF_INPUT in config:
            raise cv.Invalid(f"{CONF_INPUT} is only valid when {CONF_MODE}: {MODE_RELAY_LATCH}")
        if CONF_RELAY_OUTPUT in config:
            raise cv.Invalid(f"{CONF_RELAY_OUTPUT} is only valid when {CONF_MODE}: {MODE_RELAY_LATCH}")

    elif mode == MODE_RELAY_LATCH:
        if CONF_INPUT not in config:
            raise cv.Invalid(f"{CONF_INPUT} is required when {CONF_MODE}: {MODE_RELAY_LATCH}")
        if CONF_RELAY_OUTPUT not in config:
            raise cv.Invalid(f"{CONF_RELAY_OUTPUT} is required when {CONF_MODE}: {MODE_RELAY_LATCH}")
        if CONF_DETECTION in config:
            raise cv.Invalid(f"{CONF_DETECTION} is not used when {CONF_MODE}: {MODE_RELAY_LATCH}")
        if CONF_PULSE_OUTPUT in config:
            raise cv.Invalid(f"{CONF_PULSE_OUTPUT} is not used when {CONF_MODE}: {MODE_RELAY_LATCH}; use {CONF_RELAY_OUTPUT}")

    return config


CONFIG_SCHEMA = cv.All(
    _migrate_feedback_inverted,
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Teleruptor),
            cv.Optional(CONF_MODE, default=MODE_PULSE_FEEDBACK): cv.one_of(
                MODE_PULSE_FEEDBACK,
                MODE_PULSE_OPTIMISTIC,
                MODE_RELAY_LATCH,
                lower=True,
            ),
            cv.Optional(CONF_DETECTION): BINARY_INPUT_SCHEMA,
            cv.Optional(CONF_INPUT): BUTTON_INPUT_SCHEMA,
            cv.Optional(CONF_PULSE_OUTPUT): BINARY_OUTPUT_SCHEMA,
            cv.Optional(CONF_RELAY_OUTPUT): BINARY_OUTPUT_SCHEMA,
            cv.Optional(CONF_PULSE_DURATION, default="200ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_COMMAND_COOLDOWN, default="1000ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_FEEDBACK_TIMEOUT, default="3000ms"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_STATE_INVERTED, default=False): cv.boolean,
            cv.Optional(CONF_FEEDBACK_INVERTED): cv.boolean,
            cv.Optional(CONF_SYNC_ON_BOOT, default=True): cv.boolean,
            cv.Optional(CONF_DEFAULT_STATE, default=False): cv.boolean,
            cv.Optional(CONF_ALLOW_PULSE_WITHOUT_FEEDBACK, default=False): cv.boolean,
            cv.Optional(CONF_BLOCK_AFTER_FEEDBACK_TIMEOUT, default=False): cv.boolean,
            cv.Optional(CONF_UNSTABLE_DETECTION, default={}): UNSTABLE_DETECTION_SCHEMA,
            cv.Optional(CONF_LIGHT): TELERUPTOR_LIGHT_SCHEMA,
            cv.Optional(CONF_SWITCH): TELERUPTOR_SWITCH_SCHEMA,
            cv.Optional(CONF_BUTTON): TELERUPTOR_BUTTON_SCHEMA,
            cv.Optional(CONF_BINARY_SENSOR): TELERUPTOR_BINARY_SENSOR_SCHEMA,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_mode_config,
)


async def _setup_binary_input(var, config, key, set_component, set_pin, set_debounce):
    if key not in config:
        return
    input_config = config[key]
    if isinstance(input_config, dict):
        pin = await cg.gpio_pin_expression(input_config[CONF_PIN])
        cg.add(set_pin(pin))
        cg.add(set_debounce(input_config[CONF_DEBOUNCE]))
    else:
        binary = await cg.get_variable(input_config)
        cg.add(set_component(binary))


async def _setup_binary_output(var, config, key, set_component, set_pin):
    if key not in config:
        return
    output_config = config[key]
    if isinstance(output_config, dict):
        pin = await cg.gpio_pin_expression(output_config[CONF_PIN])
        cg.add(set_pin(pin))
    else:
        output_component = await cg.get_variable(output_config)
        cg.add(set_component(output_component))


async def register_teleruptor_light(config, parent):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    cg.add(var.set_parent(parent))
    await light.register_light(var, config)


async def register_teleruptor_switch(config, parent):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)
    cg.add(var.set_parent(parent))


async def register_teleruptor_button(config, parent):
    var = await button.new_button(config)
    await cg.register_component(var, config)
    cg.add(var.set_parent(parent))


async def register_teleruptor_binary_sensor(config, parent):
    var = await binary_sensor.new_binary_sensor(config)
    await cg.register_component(var, config)
    cg.add(var.set_parent(parent))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_mode(MODE_TO_INT[config[CONF_MODE]]))
    cg.add(var.set_default_state(config[CONF_DEFAULT_STATE]))

    await _setup_binary_input(
        var,
        config,
        CONF_DETECTION,
        var.set_detection,
        var.set_detection_pin,
        var.set_detection_debounce,
    )
    await _setup_binary_input(
        var,
        config,
        CONF_INPUT,
        var.set_input,
        var.set_input_pin,
        var.set_input_debounce,
    )
    await _setup_binary_output(
        var,
        config,
        CONF_PULSE_OUTPUT,
        var.set_pulse_output,
        var.set_pulse_pin,
    )
    await _setup_binary_output(
        var,
        config,
        CONF_RELAY_OUTPUT,
        var.set_relay_output,
        var.set_relay_pin,
    )

    cg.add(var.set_pulse_duration(config[CONF_PULSE_DURATION]))
    cg.add(var.set_command_cooldown(config[CONF_COMMAND_COOLDOWN]))
    cg.add(var.set_feedback_timeout(config[CONF_FEEDBACK_TIMEOUT]))
    cg.add(var.set_state_inverted(config[CONF_STATE_INVERTED]))
    cg.add(var.set_sync_on_boot(config[CONF_SYNC_ON_BOOT]))
    cg.add(var.set_allow_pulse_without_feedback(config[CONF_ALLOW_PULSE_WITHOUT_FEEDBACK]))
    cg.add(var.set_block_after_feedback_timeout(config[CONF_BLOCK_AFTER_FEEDBACK_TIMEOUT]))

    unstable = config[CONF_UNSTABLE_DETECTION]
    cg.add(var.set_unstable_detection_enabled(unstable[CONF_ENABLED]))
    cg.add(var.set_unstable_detection_window(unstable[CONF_WINDOW]))
    cg.add(var.set_unstable_detection_max_changes(unstable[CONF_MAX_CHANGES]))
    cg.add(var.set_unstable_detection_block_commands(unstable[CONF_BLOCK_COMMANDS]))

    if CONF_LIGHT in config:
        await register_teleruptor_light(config[CONF_LIGHT], var)
    if CONF_SWITCH in config:
        await register_teleruptor_switch(config[CONF_SWITCH], var)
    if CONF_BUTTON in config:
        await register_teleruptor_button(config[CONF_BUTTON], var)
    if CONF_BINARY_SENSOR in config:
        await register_teleruptor_binary_sensor(config[CONF_BINARY_SENSOR], var)
