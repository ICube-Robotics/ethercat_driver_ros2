#!/usr/bin/env python3
# Copyright 2024 ICUBE Laboratory, University of Strasbourg
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Author: Manuel YGUEL (yguel.robotics@gmail.com)

# Executable python3 script to update the configuration of a slave device

from lxml import etree
import os
import sys
import click


def prepend_slave_config_xml_tag(xml_file, prepend_path, output_file=None):
    """
    Update the slave_config path in the xml file by prepending the build path.

    Open the xml file, search for the tags param in ec_module
    that have name="slave_config" as attribute and update the slave_config
    file path by appending the prepend_path to the existing path
    """
    tree = etree.parse(xml_file)
    find_all_param = tree.findall('.//param')
    for param in find_all_param:
        if param.attrib.get('name') == 'slave_config':
            print(f"Found slave_config path: {param.text}")
            param.text = os.path.join(prepend_path, param.text)
            print(f"Updated slave_config path: {param.text}")
        if param.attrib.get('name') == 'safety':
            print(f"Found safety path: {param.text}")
            param.text = os.path.join(prepend_path, param.text)
            print(f"Updated safety path: {param.text}")
    if output_file:
        tree.write(output_file, pretty_print=True,
                   xml_declaration=True, encoding='UTF-8')
    else:
        tree.write(xml_file, pretty_print=True,
                   xml_declaration=True, encoding='UTF-8')


@click.command()
@click.argument('xml_file', type=click.Path(exists=True))
@click.option('--prepend_path', '-p', type=click.Path(exists=True),
              help='Prepend path', required=True)
@click.option('--output_file', '-o', type=click.Path(exists=False),
              help='Output file', default=None)
def main(xml_file, prepend_path, output_file):
    prepend_slave_config_xml_tag(xml_file, prepend_path, output_file)
    msg = f"Updated the slave config path in {xml_file} with {prepend_path}"
    if output_file:
        msg += f" and saved to {output_file}"
    else:
        msg += f" and saved to {xml_file}"
    print(msg)
    return 0


if __name__ == '__main__':
    sys.exit(main())
