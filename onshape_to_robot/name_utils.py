import re

def clean_configuration_suffix(name):
    """
    Remove configuration suffixes from names
    Examples:
    - part_name__configuration_default -> part_name
    - part_name__configuration_screwless_2 -> part_name
    - part_name_configuration_default -> part_name
    """
    # Match either double underscore or single underscore followed by 'configuration_'
    # and capture everything until the end of the string or next underscore
    return re.sub(r'(__|_)configuration_[^_]*(_\d+)?', '', name)


def clean_name(name, clean_config=False):
    """
    Clean a name according to configuration options
    
    Args:
        name (str): The name to clean
        clean_config (bool): Whether to remove configuration suffixes
    
    Returns:
        str: The cleaned name
    """
    result = name
    
    # Then apply configuration suffix removal if enabled
    if clean_config:
        result = clean_configuration_suffix(result)
    
    return result
