"""Validate shared lifecycle and topic metadata before generating SDK files."""
import re


def validate_metadata(schema, sizes):
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
    if not topics and not ({'SUBSCRIBE', 'UNSUBSCRIBE', 'TELEMETRY_EVENT'} & commands.keys()):
        return
    if not {'SUBSCRIBE', 'UNSUBSCRIBE', 'TELEMETRY_EVENT'} <= commands.keys():
        raise ValueError('Schema is missing runtime telemetry commands')
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
    if not {'ENCODER_ODOMETRY', 'PLATFORM_ODOMETRY', 'TEMPERATURE', 'IMU'} <= names:
        raise ValueError('Schema is missing runtime topics')
    for command in commands.values():
        for prop in command.get('properties', []):
            if 'enum' in prop and (prop['enum'] != 'telemetry_topic' or prop['type'] != 'uint8_t'):
                raise ValueError('Invalid topic enum reference')
    prefix = [('topic_id', 'uint8_t', 'telemetry_topic'), ('index', 'uint8_t', None)]
    for name in ('SUBSCRIBE', 'UNSUBSCRIBE', 'TELEMETRY_EVENT'):
        expected = prefix + ([('interval_ms', 'uint32_t', None)] if name == 'SUBSCRIBE' else [])
        if [(p['name'], p['type'], p.get('enum')) for p in commands[name].get('properties', [])] != expected:
            raise ValueError('Invalid telemetry envelope: ' + name)
    if commands['TELEMETRY_EVENT'].get('topic_payload') != {'discriminator': 'topic_id'}:
        raise ValueError('Invalid telemetry topic discriminator')


def retirement_notice(command):
    retired = command['retired']
    return (f"Retired since protocol {retired['since']}. Use {retired['replacement']}. "
            f"Kept for compatibility; eligible for removal in protocol {retired['removal_major']}.0.0.")
