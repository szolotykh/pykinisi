"""Validate shared lifecycle and topic metadata before generating SDK files."""
import re



def validate_subscription_settings(schema):
    commands = {c['command']: c for c in schema.get('commands', [])}
    if 'SUBSCRIBE_CONFIGURED' not in commands:
        if schema.get('subscription_triggers') or schema.get('subscription_deliveries'):
            raise ValueError('Subscription settings need SUBSCRIBE_CONFIGURED')
        return
    for key, expected in [('subscription_triggers', [(0, 'PERIODIC'), (1, 'ON_EVENT')]),
                          ('subscription_deliveries', [(0, 'ALL'), (1, 'ON_CHANGE')])]:
        values = schema.get(key, [])
        if [(v.get('id'), v.get('name')) for v in values] != expected or any(type(v.get('id')) is not int for v in values):
            raise ValueError('Invalid ' + key + ' catalog')
    props = commands['SUBSCRIBE_CONFIGURED'].get('properties', [])
    expected = [('topic_id', 'uint8_t', 'telemetry_topic'), ('index', 'uint8_t', None),
                ('poll_interval_ms', 'uint32_t', None), ('trigger', 'uint8_t', 'subscription_trigger'),
                ('delivery', 'uint8_t', 'subscription_delivery'), ('max_silence_ms', 'uint32_t', None)]
    if [(p['name'], p['type'], p.get('enum')) for p in props] != expected:
        raise ValueError('Invalid SUBSCRIBE_CONFIGURED envelope')
    for topic in schema.get('topics', []):
        if type(topic.get('event_supported')) is not bool:
            raise ValueError('Topic needs boolean event_supported')

def validate_metadata(schema, sizes):
    validate_subscription_settings(schema)
    def version(value):
        if isinstance(value, str) and re.fullmatch(r'(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)', value):
            return tuple(map(int, value.split('.')))
        return None

    commands = {c['command']: c for c in schema['commands']}
    for name, command in commands.items():
        if 'retired' not in command:
            continue
        retired = command['retired']
        if not isinstance(retired, dict) or set(retired) != {'since', 'replacement', 'removal_major'}:
            raise ValueError(f'{name}: invalid retired metadata')
        since, current = version(retired['since']), version(schema['version'])
        if since is None or current is None or since > current:
            raise ValueError(f'{name}: invalid retired.since')
        removal = retired['removal_major']
        if type(removal) is not int or not since[0] < removal <= 255:
            raise ValueError(f'{name}: invalid retired.removal_major')
        replacement = retired['replacement']
        target = commands.get(replacement) if isinstance(replacement, str) else None
        if not target or replacement == name or target['direction'] != command['direction'] or 'retired' in target:
            raise ValueError(f'{name}: invalid retired.replacement')

    topics = schema.get('topics', [])
    # Small codec-only fixtures/older schemas can omit the entire telemetry family.
    if not topics and not ({'SUBSCRIBE', 'SUBSCRIBE_WITH_MODE', 'UNSUBSCRIBE', 'TELEMETRY_EVENT'} & commands.keys()):
        return
    if not {'SUBSCRIBE', 'SUBSCRIBE_WITH_MODE', 'UNSUBSCRIBE', 'TELEMETRY_EVENT'} <= commands.keys():
        raise ValueError('Schema is missing runtime telemetry commands')
    modes = schema.get('subscription_modes', [])
    if [(m.get('id'), m.get('name')) for m in modes] != [(0, 'PERIODIC'), (1, 'ON_CHANGE')] or any(type(m.get('id')) is not int for m in modes):
        raise ValueError('Invalid subscription mode catalog')
    objects = {o['name']: o for o in schema['objects']}
    ids, names = set(), set()
    for topic in topics:
        code, name = topic.get('id'), topic.get('name')
        if type(code) is not int or not 1 <= code <= 255 or code in ids:
            raise ValueError('Invalid or duplicate topic ID')
        if not isinstance(name, str) or not re.fullmatch(r'[A-Z][A-Z0-9_]*', name) or name in names:
            raise ValueError('Invalid or duplicate topic name')
        ids.add(code); names.add(name)
        obj = objects.get(topic.get('payload_type'))
        if not obj or any(p['type'] not in sizes for p in obj['properties']):
            raise ValueError('Topic payload must reference a fixed-size object')
        if sum(sizes[p['type']] for p in obj['properties']) > 249:
            raise ValueError('Topic payload exceeds frame limit')
        if [(p['name'], p['type']) for p in obj['properties'][:3]] != [
                ('timestamp_us', 'uint64_t'), ('clock_mode', 'uint8_t'), ('clock_quality', 'uint8_t')]:
            raise ValueError('Topic payload must start with timestamp and clock metadata')
        for field, low, high in [('index_max', 0, 255), ('min_interval_ms', 2, 3600000)]:
            if type(topic.get(field)) is not int or not low <= topic[field] <= high:
                raise ValueError('Invalid topic ' + field)
    if not {'ENCODER_ODOMETRY', 'PLATFORM_ODOMETRY', 'TEMPERATURE', 'IMU', 'MOTOR_CURRENT', 'MOTOR_FAULTS'} <= names:
        raise ValueError('Schema is missing runtime topics')
    for command in commands.values():
        for prop in command.get('properties', []):
            if 'enum' in prop and (prop['enum'] not in ('telemetry_topic', 'subscription_mode', 'subscription_trigger', 'subscription_delivery') or prop['type'] != 'uint8_t'):
                raise ValueError('Invalid topic enum reference')
    prefix = [('topic_id', 'uint8_t', 'telemetry_topic'), ('index', 'uint8_t', None)]
    for name in ('SUBSCRIBE', 'SUBSCRIBE_WITH_MODE', 'UNSUBSCRIBE', 'TELEMETRY_EVENT'):
        expected = prefix + ([('interval_ms', 'uint32_t', None)] if name in ('SUBSCRIBE', 'SUBSCRIBE_WITH_MODE') else [])
        if name == 'SUBSCRIBE_WITH_MODE':
            expected += [('mode', 'uint8_t', 'subscription_mode')]
        if [(p['name'], p['type'], p.get('enum')) for p in commands[name].get('properties', [])] != expected:
            raise ValueError('Invalid telemetry envelope: ' + name)
    if commands['TELEMETRY_EVENT'].get('topic_payload') != {'discriminator': 'topic_id'}:
        raise ValueError('Invalid telemetry topic discriminator')


def retirement_notice(command):
    retired = command['retired']
    return (f"Retired since protocol {retired['since']}. Use {retired['replacement']}. "
            f"Kept for compatibility; eligible for removal in protocol {retired['removal_major']}.0.0.")
