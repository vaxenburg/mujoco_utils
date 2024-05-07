"""MJCF utilities."""

from lxml import etree


def mjcf2xml(mjcf_model: 'mjcf.RootElement',
             output_file_name: str | None = None,
             precision: int = 5,
             zero_threshold: float = 1e-7) -> str | None:
    """Generate clean XML string from mjcf_model and optionally save to file.

    Args:
        mjcf_model: MJCF model to export.
        output_file_name: Optional file name, if provided, the generated XML
            string be saved to this file.
        precision: Number of digits to output for floating point quantities.
        zero_threshold: When outputting XML, floating point quantities whose
            absolute value falls below this threshold will be treated as zero.

    Returns:
        XML string or saves it to file.
    """
    
    # Dirty export to string first.
    xml_string = mjcf_model.to_xml_string(
        'float', precision=precision, zero_threshold=zero_threshold)
    
    # Remove empty default.
    root = etree.XML(xml_string, etree.XMLParser(remove_blank_text=True))
    default_elem = root.find('default')
    root.insert(3, default_elem[0])
    root.remove(default_elem)

    # Remove hashes from filenames.
    meshes = [mesh for mesh in root.find('asset').iter() if mesh.tag == 'mesh']
    skins = [skin for skin in root.find('asset').iter() if skin.tag == 'skin']
    things = meshes + skins
    for thing in things:
        name, extension = thing.get('file').split('.')
        thing.set('file', '.'.join((name[:-41], extension)))

    # Get string from lxml and remove class="/".
    xml_string = etree.tostring(root, pretty_print=True)
    xml_string = xml_string.replace(b' class="/"', b'')

    # Remove gravcomp="0".
    xml_string = xml_string.replace(b' gravcomp="0"', b'')

    # Insert spaces between top level elements.
    lines = xml_string.splitlines()
    newlines = []
    for line in lines:
        newlines.append(line)
        if line.startswith(b'  <'):
            if line.startswith(b'  </') or line.endswith(b'/>'):
                newlines.append(b'')
    newlines.append(b'')
    xml_string = b'\n'.join(newlines)

    # Save generated XML string to file or return XML string.
    if output_file_name is not None:
        with open(output_file_name, 'wb') as f:
            f.write(xml_string)
    else:
        return xml_string


def get_mjcf_tree(element: 'mjcf.Element',
                  bodies_only: bool = False) -> dict:
    """Returns dict representing kinematic tree of mjcf model.
    
    Example:
        tree = get_mjcf_tree(mjcf_model.worldbody)
        print_tree(tree)
    """
    tree = {}
    if not element._children:
        return ''
    if bodies_only:
        n_bodies = len([child for child in element._children 
                        if child.tag == 'body'])
        if n_bodies == 0:
            return ''
    for child in element._children:
        if bodies_only and child.tag != 'body':
            continue
        tree[f'{child.tag}: {child.name}'] = get_mjcf_tree(child, bodies_only)
    return tree
