import adsk.core
import adsk.fusion
import adsk.cam
import traceback
import os


class FusionURDF:
    def __init__(self):
        self.ui = None
        self.rootComp = None
        self.joints = []
        self.asbuilt_joints = []
        self.original_link_names = []
        self.new_link_names = []
        self.origins = []
        self.children = []
        self.old_origin_x = 0.0
        self.old_origin_y = 0.0
        self.old_origin_z = 0.0
    
    def process_URDF(self):
        xmlHeader = """<?xml version = "1.0" ?>\n"""
        robotHeader = """<robot name = "%s">\n"""
        robotFooter = """\n</robot>"""

        try:
            app = adsk.core.Application.get()
            ui = app.userInterface
            self.ui = ui
            design = adsk.fusion.Design.cast(app.activeProduct)
            exporter = design.exportManager
            rootComp = design.rootComponent
            self.rootComp = rootComp
            if not rootComp.occurrences:
                self.ui.messageBox('No components found. Exiting...')
                return
            if not rootComp.joints and not rootComp.asBuiltJoints:
                self.ui.messageBox('No joints found. Exiting...')
                return
            if not any('base_link' in link.name for link in rootComp.occurrences):
                self.ui.messageBox(
                    'Component named base_link not found. Please add one. Exiting...')
                return
            robotName = self.formatName(rootComp.name)
            folderOpener = self.ui.createFolderDialog()
            folderOpener.title = 'Select folder to save URDF'
            dialogResult = folderOpener.showDialog()
            if dialogResult != adsk.core.DialogResults.DialogOK:
                return
            folderPath = folderOpener.folder
            try:
                os.mkdir(os.path.join(folderPath, robotName))
                os.mkdir(os.path.join(folderPath, robotName, 'urdf'))
                os.mkdir(os.path.join(folderPath, robotName, 'meshes'))
            except:
                returnValue, cancelled = self.ui.inputBox(
                    'Folder already exists. Do you want to overwrite it? This can lead to unstable \
                    behavior. Enter Y or N', 'Warning', 'N')
                if returnValue.upper() == 'Y':
                    pass
                elif returnValue.upper() == 'N' or cancelled:
                    return
                else:
                    self.ui.messageBox('Invalid input. Exiting...')
                    return

            # Write URDF file header
            urdfFile = open(os.path.join(
                folderPath, robotName, 'urdf', robotName + '.urdf'), 'w')
            urdfFile.write(xmlHeader)
            urdfFile.write(robotHeader % robotName)

            for joint in rootComp.joints:
                self.joints.append(joint)

            for joint in rootComp.asBuiltJoints:
                self.asbuilt_joints.append(joint)

            self.getJointOrigins()
            self.ui.messageBox(str(self.origins))
            self.ui.messageBox(str(self.children))

            # Write links and export meshes
            all_occurences = rootComp.occurrences
            link_list = [occs for occs in all_occurences]
            for link in link_list:
                transform = adsk.core.Matrix3D.create()
                new_comp = all_occurences.addNewComponent(transform)

                if link.component.name == 'base_link':
                    link.component.name = 'old_component'
                    new_comp.component.name = 'base_link'
                else:
                    new_comp.component.name = 'new_' + self.formatName(link.component.name)

                self.original_link_names.append(link.component.name)
                self.new_link_names.append(new_comp.component.name)

                bodies = link.bRepBodies
                for i in range(bodies.count):
                    body = bodies.item(i)
                    body.copyToComponent(new_comp)

                urdfFile.write(self.fillLinkTemplate(new_comp))
                self.ui.messageBox(new_comp.component.name)
                mesh_name = os.path.join('meshes', new_comp.component.name + '.stl')
                meshPath = os.path.join(folderPath, robotName, mesh_name)
                stlExportOptions = exporter.createSTLExportOptions(new_comp, meshPath)
                stlExportOptions.sendToPrintUtility = False
                stlExportOptions.isBinaryFormat = True
                stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                exporter.execute(stlExportOptions)

            # Write joints
            # 0: Fixed, 1: Revolute, 2: Prismatic, 3: Continuous
            for joint in rootComp.joints:
                hasRotationLimits = joint.jointMotion.jointType == 1 and (
                    joint.jointMotion.rotationLimits.isMinimumValueEnabled or
                    joint.jointMotion.rotationLimits.isMaximumValueEnabled)
                if not hasRotationLimits and joint.jointMotion.jointType == 1:
                    urdfFile.write(self.fillJointTemplate(
                        joint, 3, False))
                else:
                    urdfFile.write(self.fillJointTemplate(
                        joint, joint.jointMotion.jointType, False))

            for joint in rootComp.asBuiltJoints:
                hasRotationLimits = joint.jointMotion.jointType == 1 and (
                    joint.jointMotion.rotationLimits.isMinimumValueEnabled or
                    joint.jointMotion.rotationLimits.isMaximumValueEnabled)
                if not hasRotationLimits and joint.jointMotion.jointType == 1:
                    urdfFile.write(self.fillJointTemplate(
                        joint, 3, True))
                else:
                    urdfFile.write(self.fillJointTemplate(
                        joint, joint.jointMotion.jointType, True))

            # Write footer
            urdfFile.write(robotFooter)
            self.ui.messageBox('Exported URDF to ' + folderPath)

        except:
            if ui:
                self.ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

    @staticmethod
    def formatName(name: str) -> str:
        if 'base_link' in name:
            return 'base_link'
        else:
            return name.translate(str.maketrans(' :()<>', '______'))

    @staticmethod
    def getTemplate(templateName: str) -> str:
        LINK = """
        <link name = "%s">
            <visual>
                <origin xyz = "%f %f %f" rpy = "0 0 0"/>
                <geometry>
                    <mesh filename = "../%s" scale = "0.001 0.001 0.001"/>
                </geometry>
            </visual>
            <collision>
                <origin xyz = "%f %f %f" rpy = "0 0 0"/>
                <geometry>
                    <mesh filename = "../%s" scale = "0.001 0.001 0.001"/>
                </geometry>
            </collision>
            <inertial>
                <origin xyz = "%f %f %f" rpy = "0 0 0"/>
                <mass value = "%f"/>
                <inertia ixx = "%f" ixy = "%f" ixz = "%f" iyy = "%f" iyz = "%f" izz = "%f"/>
            </inertial>
        </link>
        """
        CONTINUOUS = """
        <joint name = "%s" type = "continuous">
            <origin xyz = "%f %f %f" rpy = "0 0 0"/>
            <parent link = "%s"/>
            <child link = "%s"/>
            <axis xyz = "%f %f %f"/>
        </joint>
        """
        FIXED = """
        <joint name = "%s" type = "fixed">
            <origin xyz = "%f %f %f" rpy = "0 0 0"/>
            <parent link = "%s"/>
            <child link = "%s"/>
        </joint>
        """
        REVOLUTE = """
        <joint name = "%s" type = "revolute">
            <origin xyz = "%f %f %f" rpy = "0 0 0"/>
            <parent link = "%s"/>
            <child link = "%s"/>
            <axis xyz = "%f %f %f"/>
            <limit lower = "%f" upper = "%f"/>
        </joint>
        """
        PRISMATIC = """
        <joint name = "%s" type = "prismatic">
            <origin xyz = "%f %f %f" rpy = "0 0 0"/>
            <parent link = "%s"/>
            <child link = "%s"/>
            <axis xyz = "%f %f %f"/>
            <limit lower = "%f" upper = "%f"/>
        </joint>
        """
        return locals()[templateName.upper()]

    def getJointOrigins(self):
        for joint in self.rootComp.joints:
            joint_origin = joint.geometryOrOriginOne
            joint_child = joint.occurrenceOne.component.name
            self.children.append(joint_child)
            self.origins.append([joint_origin.origin.x, 
                                 joint_origin.origin.y, 
                                 joint_origin.origin.z])
            
        for joint in self.rootComp.asBuiltJoints:
            joint_origin = joint.geometry
            self.children.append(joint_child)
            self.origins.append([joint_origin.origin.x,
                                 joint_origin.origin.y,
                                 joint_origin.origin.z])


    def fillLinkTemplate(self, link: adsk.fusion.Occurrence) -> str:
        link_origin_x, link_origin_y, link_origin_z = 0, 0, 0
        new_to_original_links = dict(zip(self.new_link_names, self.original_link_names))
        original_link_name = new_to_original_links[link.component.name]
        
        link_name_to_origin = dict(zip(self.children, self.origins))
        if link.component.name != 'base_link':
            link_origin_x = -link_name_to_origin[original_link_name][0]
            link_origin_y = -link_name_to_origin[original_link_name][1]
            link_origin_z = -link_name_to_origin[original_link_name][2]

        _, xx, yy, zz, xy, yz, xz = link.getPhysicalProperties().getXYZMomentsOfInertia()
        cm_to_m = 0.01
        kgcm2_to_kgm2 = 1e-6
        parsed_name = self.formatName(link.component.name)
        mesh_name = 'meshes/' + parsed_name + '.stl'
        linkTemplate = self.getTemplate('link')
        return linkTemplate % (parsed_name,
                            link_origin_x * cm_to_m,
                            link_origin_y * cm_to_m,
                            link_origin_z * cm_to_m,
                            mesh_name,
                            link_origin_x * cm_to_m,
                            link_origin_y * cm_to_m,
                            link_origin_z * cm_to_m,
                            mesh_name,
                            link_origin_x * cm_to_m,
                            link_origin_y * cm_to_m,
                            link_origin_z * cm_to_m,
                            link.getPhysicalProperties().mass,
                            xx * kgcm2_to_kgm2,
                            xy * kgcm2_to_kgm2,
                            xz * kgcm2_to_kgm2,
                            yy * kgcm2_to_kgm2,
                            yz * kgcm2_to_kgm2,
                            zz * kgcm2_to_kgm2,)


    def fillJointTemplate(self, joint: adsk.fusion.Joint, jointType: int, asBuilt: bool) -> str:
        jointDict = {0: 'fixed', 1: 'revolute', 2: 'prismatic', 3: 'continuous'}
        jointTypeStr = jointDict[jointType]
        jointTemplate = self.getTemplate(jointTypeStr)

        cm_to_m = 0.01

        original_to_new_links = dict(zip(self.original_link_names, self.new_link_names))

        original_joint_link_one = self.formatName(joint.occurrenceOne.component.name)
        original_joint_link_two = self.formatName(joint.occurrenceTwo.component.name)

        new_joint_link_one = original_to_new_links[original_joint_link_one]
        new_joint_link_two = original_to_new_links[original_joint_link_two]

        childLink = self.formatName(new_joint_link_one)
        if childLink == 'base_link':
            raise ValueError(
                'base_link cannot be a child link. Reverse the selection of the joint.')
        parentLink = self.formatName(new_joint_link_two)

        if asBuilt:
            joint_origin = joint.geometry
        else:
            joint_origin = joint.geometryOrOriginOne

        if self.old_origin_x:
            new_origin_x = joint_origin.origin.x - self.old_origin_x
            new_origin_y = joint_origin.origin.y - self.old_origin_y
            new_origin_z = joint_origin.origin.z - self.old_origin_z
        else:
            new_origin_x = joint_origin.origin.x
            new_origin_y = joint_origin.origin.y
            new_origin_z = joint_origin.origin.z

        self.old_origin_x += new_origin_x
        self.old_origin_y += new_origin_y
        self.old_origin_z += new_origin_z
        self.origins.append([self.old_origin_x, self.old_origin_y, self.old_origin_z])

        if jointTypeStr == 'continuous':
            joint_axis = joint.jointMotion.rotationAxisVector
            return jointTemplate % (joint.name,
                                    new_origin_x * cm_to_m,
                                    new_origin_y * cm_to_m,
                                    new_origin_z * cm_to_m,
                                    parentLink,
                                    childLink,
                                    joint_axis.x,
                                    joint_axis.y,
                                    joint_axis.z)
        elif jointTypeStr == 'fixed':
            return jointTemplate % (joint.name,
                                    new_origin_x * cm_to_m,
                                    new_origin_y * cm_to_m,
                                    new_origin_z * cm_to_m,
                                    parentLink,
                                    childLink)
        elif jointTypeStr == 'revolute':
            joint_axis = joint.jointMotion.rotationAxisVector
            joint_limits = joint.jointMotion.rotationLimits
            return jointTemplate % (joint.name,
                                    new_origin_x * cm_to_m,
                                    new_origin_y * cm_to_m,
                                    new_origin_z * cm_to_m,
                                    parentLink,
                                    childLink,
                                    joint_axis.x,
                                    joint_axis.y,
                                    joint_axis.z,
                                    joint_limits.minimumValue,
                                    joint_limits.maximumValue)
        elif jointTypeStr == 'prismatic':
            joint_axis = joint.jointMotion.slideDirectionVector
            joint_limits = joint.jointMotion.slideLimits
            return jointTemplate % (joint.name,
                                    new_origin_x * cm_to_m,
                                    new_origin_y * cm_to_m,
                                    new_origin_z * cm_to_m,
                                    parentLink,
                                    childLink,
                                    joint_axis.x,
                                    joint_axis.y,
                                    joint_axis.z,
                                    joint_limits.minimumValue,
                                    joint_limits.maximumValue)
        else:
            raise ValueError('Invalid joint type')

def run(context):
    fusion_urdf_loader = FusionURDF()
    fusion_urdf_loader.process_URDF()
