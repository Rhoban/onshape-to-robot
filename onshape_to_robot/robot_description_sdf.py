
# class RobotSDF(RobotDescription):
#     def __init__(self, name):
#         super().__init__(name)
#         self.ext = 'sdf'
#         self.relative = False
#         self.json = {
#             "sdf": {
#                 "version": "1.6",
#                 "model": {
#                     "name": self.robotName,
#                     "link": [],
#                     "joint": [],
#                 }
#             }
#         }
#         self.append('<sdf version="1.6">')
#         self.append('<model name="'+self.robotName + '">')
#         pass

#     def addFixedJoint(self, parent, child, matrix, name=None):
#         if name is None:
#             name = parent+'_'+child+'_fixing'

#         self.append('<joint name="'+name+'" type="fixed">')
#         self.append(pose(matrix))
#         self.append('<parent>'+parent+'</parent>')
#         self.append('<child>'+child+'</child>')
#         self.append('</joint>')
#         self.append('')

#     def addDummyLink(self, name, visualMatrix=None, visualSTL=None, visualColor=None):
#         self.append('<link name="'+name+'">')
#         self.append('<pose>0 0 0 0 0 0</pose>')
#         self.append('<inertial>')
#         self.append('<pose>0 0 0 0 0 0</pose>')
#         self.append('<mass>1e-9</mass>')
#         self.append('<inertia>')
#         self.append(
#             '<ixx>0</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0</iyy><iyz>0</iyz><izz>0</izz>')
#         self.append('</inertia>')
#         self.append('</inertial>')
#         if visualSTL is not None:
#             self.addSTL(visualMatrix, visualSTL, visualColor,
#                         name+"_visual", "visual")
#         self.append('</link>')

#     def startLink(self, name, matrix):
#         self._link_name = name
#         self.resetLink()
#         self.append('<link name="'+name+'">')
#         self.append(pose(matrix, name))

#     def endLink(self):
#         mass, com, inertia = self.linkDynamics()

#         for node in ['visual', 'collision']:
#             if self._mesh[node] is not None:
#                 color = self._color / self._color_mass
#                 filename = self._link_name+'_'+node+'.stl'
#                 stl_combine.save_mesh(
#                     self._mesh[node], self.meshDir+'/'+filename)
#                 if self.shouldSimplifySTLs(node):
#                     stl_combine.simplify_stl(
#                         self.meshDir+'/'+filename, self.maxSTLSize)
#                 self.addSTL(np.identity(4), filename, color, self._link_name, 'visual')

#         self.append('<inertial>')
#         self.append('<pose frame="'+self._link_name +
#                     '_frame">%.20g %.20g %.20g 0 0 0</pose>' % (com[0], com[1], com[2]))
#         self.append('<mass>%.20g</mass>' % mass)
#         self.append('<inertia><ixx>%.20g</ixx><ixy>%.20g</ixy><ixz>%.20g</ixz><iyy>%.20g</iyy><iyz>%.20g</iyz><izz>%.20g</izz></inertia>' %
#                     (inertia[0, 0], inertia[0, 1], inertia[0, 2], inertia[1, 1], inertia[1, 2], inertia[2, 2]))
#         self.append('</inertial>')

#         if self.useFixedLinks:
#             self.append(
#                 '<visual><geometry><box><size>0 0 0</size></box></geometry></visual>')

#         self.append('</link>')
#         self.append('')

#         if self.useFixedLinks:
#             n = 0
#             for visual in self._visuals:
#                 n += 1
#                 visual_name = '%s_%d' % (self._link_name, n)
#                 self.addDummyLink(visual_name, visual[0], visual[1], visual[2])
#                 self.addJoint('fixed', self._link_name, visual_name,
#                               np.eye(4), visual_name+'_fixing', None)

#     def addFrame(self, name, matrix):
#         # Adding a dummy link
#         self.addDummyLink(name)

#         # Linking it with last link with a fixed link
#         self.addFixedJoint(self._link_name, name, matrix, name+'_frame')

#     def material(self, color):
#         m = '<material>'
#         m += '<ambient>%.20g %.20g %.20g 1</ambient>' % (color[0], color[1], color[2])
#         m += '<diffuse>%.20g %.20g %.20g 1</diffuse>' % (color[0], color[1], color[2])
#         m += '<specular>0.1 0.1 0.1 1</specular>'
#         m += '<emissive>0 0 0 0</emissive>'
#         m += '</material>'

#         return m

#     def addSTL(self, matrix, stl, color, name, node='visual'):
#         self.append('<'+node+' name="'+name+'_visual">')
#         self.append(pose(matrix))
#         self.append('<geometry>')
#         self.append('<mesh><uri>file://'+stl+'</uri></mesh>')
#         self.append('</geometry>')
#         if node == 'visual':
#             self.append(self.material(color))
#         self.append('</'+node+'>')

#     def addPart(self, matrix, stl, mass, com, inertia, color, shapes=None, name=''):
#         name = self._link_name+'_'+str(self._link_childs)+'_'+name
#         self._link_childs += 1

#         # self.append('<link name="'+name+'">')
#         # self.append(pose(matrix))

#         if stl is not None:
#             if not self.drawCollisions:
#                 if self.useFixedLinks:
#                     self._visuals.append(
#                         [matrix, self.packageName + os.path.basename(stl), color])
#                 elif self.shouldMergeSTLs('visual'):
#                     self.mergeSTL(stl, matrix, color, mass)
#                 else:
#                     self.addSTL(matrix, os.path.basename(
#                         stl), color, name, 'visual')

#             entries = ['collision']
#             if self.drawCollisions:
#                 entries.append('visual')
#             for entry in entries:
#                 if shapes is None:
#                     # We don't have pure shape, we use the mesh
#                     if self.shouldMergeSTLs(entry):
#                         self.mergeSTL(stl, matrix, color, mass, entry)
#                     else:
#                         self.addSTL(matrix, stl, color, name, entry)
#                 else:
#                     # Inserting pure shapes in the URDF model
#                     k = 0
#                     self.append('<!-- Shapes for '+name+' -->')
#                     for shape in shapes:
#                         k += 1
#                         self.append('<'+entry+' name="'+name +
#                                     '_'+entry+'_'+str(k)+'">')
#                         self.append(pose(matrix*shape['transform']))
#                         self.append('<geometry>')
#                         if shape['type'] == 'cube':
#                             self.append('<box><size>%.20g %.20g %.20g</size></box>' %
#                                         tuple(shape['parameters']))
#                         if shape['type'] == 'cylinder':
#                             self.append(
#                                 '<cylinder><length>%.20g</length><radius>%.20g</radius></cylinder>' % tuple(shape['parameters']))
#                         if shape['type'] == 'sphere':
#                             self.append(
#                                 '<sphere><radius>%.20g</radius></sphere>' % shape['parameters'])
#                         self.append('</geometry>')

#                         if entry == 'visual':
#                             self.append(self.material(color))
#                         self.append('</'+entry+'>')

#         self.addLinkDynamics(matrix, mass, com, inertia)

#     def addJoint(self, jointType, linkFrom, linkTo, transform, name, jointLimits, zAxis=[0, 0, 1]):
#         self.append('<joint name="'+name+'" type="'+jointType+'">')
#         self.append(pose(transform))
#         self.append('<parent>'+linkFrom+'</parent>')
#         self.append('<child>'+linkTo+'</child>')
#         self.append('<axis>')
#         self.append('<xyz>%.20g %.20g %.20g</xyz>' % tuple(zAxis))
#         lowerUpperLimits = ''
#         if jointLimits is not None:
#             lowerUpperLimits = '<lower>%.20g</lower><upper>%.20g</upper>' % jointLimits
#         self.append('<limit><effort>%.20g</effort><velocity>%.20g</velocity>%s</limit>' %
#                     (self.jointMaxEffortFor(name), self.jointMaxVelocityFor(name), lowerUpperLimits))
#         self.append('</axis>')
#         self.append('</joint>')
#         self.append('')
#         # print('Joint from: '+linkFrom+' to: '+linkTo+', transform: '+str(transform))

#     def finalize(self):
#         self.append(self.additionalXML)
#         self.append('</model>')
#         self.append('</sdf>')
