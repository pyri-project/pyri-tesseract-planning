from pyri.plugins.blockly import PyriBlocklyPluginFactory, PyriBlocklyBlock, PyriBlocklyCategory
from typing import List, Dict, NamedTuple, TYPE_CHECKING

def _get_blocks() -> Dict[str,PyriBlocklyBlock]:
    blocks = {}
        
    blocks["tesseract_get_robot_position"] = PyriBlocklyBlock(
        name = "tesseract_get_robot_position",
        category = "Tesseract",
        doc = "Get the tesseract virtual robot position",
        json = """
        {
            "type": "tesseract_get_robot_position",
            "message0": "tesseract robot position",
            "output": null,
            "colour": 15,
            "tooltip": "get the tesseract virtual robot position",
            "helpUrl": ""
            }
        """,
        python_generator = """Blockly.Python['tesseract_get_robot_position'] = function(block) {
                                // TODO: Assemble Python into code variable.
                                var code = 'tesseract_get_robot_position()';
                                // TODO: Change ORDER_NONE to the correct strength.
                                return [code, Blockly.Python.ORDER_NONE];
                                };"""
    )

    blocks["tesseract_set_robot_position"] = PyriBlocklyBlock(
        name = "tesseract_set_robot_position",
        category = "Tesseract",
        doc = "Set the tesseract virtual robot position",
        json = """
        {
            "type": "tesseract_set_robot_position",
            "message0": "tesseract robot position %1",
            "args0": [
                {
                "type": "input_value",
                "name": "POSITION"
                }
            ],
            "previousStatement": null,
            "nextStatement": null,
            "colour": 15,
            "tooltip": "set the tesseract virtual robot position",
            "helpUrl": ""
            }
        """,
        python_generator = """Blockly.Python['tesseract_set_robot_position'] = function(block) {
                                var value_position = Blockly.Python.valueToCode(block, 'POSITION', Blockly.Python.ORDER_ATOMIC);
                                // TODO: Assemble Python into code variable.
                                var code = 'tesseract_set_robot_position(' + value_position + ')\\n';
                                return code;
                                };"""
    )    

    blocks["tesseract_sync_robot"] = PyriBlocklyBlock(
        name = "tesseract_sync_robot",
        category = "Tesseract",
        doc = "Sync virtual robot with real robot",
        json = """
        {
            "type": "tesseract_sync_robot",
            "message0": "tesseract sync robot",
            "previousStatement": null,
            "nextStatement": null,
            "colour": 15,
            "tooltip": "sync virtual robot with real robot",
            "helpUrl": ""
            }
        """,
        python_generator = """Blockly.Python['tesseract_sync_robot'] = function(block) {
                            // TODO: Assemble Python into code variable.
                            var code = 'tesseract_sync_robot()\\n';
                            return code;
                            };"""
    )

    blocks["tesseract_plan_freespace_joint_target"] = PyriBlocklyBlock(
        name = "tesseract_plan_freespace_joint_target",
        category = "Tesseract",
        doc = "Plan a collision free trajectory to joint position target",
        json = """{
                    "type": "tesseract_plan_freespace_joint_target",
                    "message0": "tesseract plan freespace trajectory with speed %1 to joint position %2",
                    "args0": [
                        {
                        "type": "field_number",
                        "name": "SPEED",
                        "value": 100,
                        "min": 0,
                        "max": 100,
                        "precision": 1
                        },
                        {
                        "type": "input_value",
                        "name": "POSITION"
                        }
                    ],
                    "output": null,
                    "colour": 15,
                    "tooltip": "plan a collision free trajectory with joint position target",
                    "helpUrl": ""
                    }
                    """,
        python_generator = """Blockly.Python['tesseract_plan_freespace_joint_target'] = function(block) {
                                var number_speed = block.getFieldValue('SPEED');
                                var value_position = Blockly.Python.valueToCode(block, 'POSITION', Blockly.Python.ORDER_ATOMIC);
                                // TODO: Assemble Python into code variable.
                                var code = 'tesseract_plan_freespace_joint_target(' + value_position + ',' + number_speed + ')';
                                // TODO: Change ORDER_NONE to the correct strength.
                                return [code, Blockly.Python.ORDER_NONE];
                                };"""
    )
    
    return blocks

def _get_categories() -> Dict[str,PyriBlocklyCategory]:
    categories = {}
    categories["Tesseract"] = PyriBlocklyCategory(
        name = "Tesseract",
        json = '{"kind": "category", "name": "Tesseract", "colour": 15 }'
    )

    return categories


class PyriTesseractBlocklyPluginFactory(PyriBlocklyPluginFactory):
    def get_plugin_name(self):
        return "pyri-tesseract-planner"

    def get_category_names(self) -> List[str]:
        return ["Tesseract"]

    def get_categories(self) -> List[PyriBlocklyCategory]:
        return _get_categories()

    def get_block_names(self) -> List[str]:
        return list(_get_blocks().keys())

    def get_block(self,name) -> PyriBlocklyBlock:
        return _get_blocks()[name]

    def get_all_blocks(self) -> Dict[str,PyriBlocklyBlock]:
        return _get_blocks()

def get_blockly_factory():
    return PyriTesseractBlocklyPluginFactory()