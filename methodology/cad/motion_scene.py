import xml.etree.ElementTree as xml

import numpy as np

from model import Inputs, Fixed, Constants
from methodology.cad import assembly, square_assembly


def numbers(values):
    return ' '.join(str(value) for value in np.asarray(values).ravel())


def add_mesh(asset, name, shape, offset_mm=(0, 0, 0)):
    vertices, triangles = shape.tessellate(Fixed.cad_geometry_tolerance, Fixed.motion_mesh_angular_tolerance)
    vertices = (np.array([tuple(vertex) for vertex in vertices]) - offset_mm) / 1000
    xml.SubElement(asset, 'mesh', name=name, vertex=numbers(vertices), face=numbers(triangles))


def scene_xml(construction, refinement=1):
    parts, array, winding, reference_piece = square_assembly.square_parts()
    properties = construction.piece_properties
    center = np.asarray(properties['center_mm']) / 1000
    tensor = np.asarray(properties['inertia_kg_m2'])
    scene = xml.Element('mujoco', model='Flying chess CAD motion interface')
    xml.SubElement(scene, 'compiler', angle='radian', inertiafromgeom='false', alignfree='false')
    xml.SubElement(scene, 'option', timestep=str(Fixed.motion_time_step / refinement),
                   gravity=numbers([0, 0, -Constants.gravity]), integrator='implicitfast', cone='elliptic',
                   iterations=str(Fixed.motion_solver_iterations), tolerance=str(Fixed.motion_solver_tolerance))
    defaults = xml.SubElement(scene, 'default')
    xml.SubElement(defaults, 'geom', friction=numbers([Fixed.resting_friction_coefficient, 0, 0]), condim='3',
                   solref=numbers([Fixed.motion_contact_time_constant, Fixed.motion_contact_damping_ratio]),
                   solimp=numbers(Fixed.motion_contact_impedance))
    asset = xml.SubElement(scene, 'asset')
    xml.SubElement(asset, 'texture', type='skybox', builtin='flat', rgb1=numbers(Fixed.motion_preview_background),
                   rgb2=numbers(Fixed.motion_preview_background),
                   width=str(Fixed.cad_preview_size[0]), height=str(Fixed.cad_preview_size[1]))
    world = xml.SubElement(scene, 'worldbody')
    board_side = array.window
    xml.SubElement(world, 'light', pos=numbers([0, 0, reference_piece.box_height / 1000 + board_side]), dir='0 0 -1')
    visual = xml.SubElement(scene, 'visual')
    xml.SubElement(visual, 'global', offwidth=str(Fixed.cad_preview_size[0]), offheight=str(Fixed.cad_preview_size[1]))
    king = xml.SubElement(world, 'body', name='king', pos=numbers(center))
    xml.SubElement(king, 'joint', name='king_free', type='free', damping='0', armature='0', frictionloss='0')
    xml.SubElement(king, 'inertial', pos='0 0 0', mass=str(properties['mass_g'] / 1000),
                   fullinertia=numbers([tensor[0, 0], tensor[1, 1], tensor[2, 2], tensor[0, 1], tensor[0, 2], tensor[1, 2]]))
    coil = construction.coil
    ring = assembly.rectangular_ring(coil.outer_width, coil.outer_length, coil.winding_radial_width,
                                     winding.coil_height, coil.corner_radius)
    add_mesh(asset, 'coil_envelope', ring)
    quaternion = array.rotation.as_quat(scalar_first=True)
    for index, placement in enumerate(array.placements):
        position = array.rotation.apply([placement['x'], placement['y'], placement['top'] - array.height])
        xml.SubElement(world, 'geom', name=f'coil_{index}', type='mesh', mesh='coil_envelope', pos=numbers(position),
                       quat=numbers(quaternion), contype='0', conaffinity='0',
                       rgba=numbers(tuple(next(part for part in parts if part.group == 'Single_layer_coils').shape.color)))
    for part in parts:
        if part.group == 'Single_layer_coils':
            continue
        moving = part.group in ('Magnets', 'King_envelope', 'Piece_skin')
        if part.group == 'Playing_surface':
            surface_center = np.array(tuple(part.shape.center())) / 1000
            surface_center[2] += Fixed.playing_surface_thickness / 2000
            shape_attributes = dict(type='plane', size=numbers([(board_side - Inputs.prototype_coil_spacing / 1000) / 2] * 3),
                                    pos=numbers(surface_center), quat=numbers(quaternion))
        elif part.group == 'Piece_skin':
            shape_attributes = dict(type='cylinder', size=numbers([reference_piece.diameter / 2000, Fixed.piece_bottom_skin / 2000]),
                                    pos=numbers(np.array([0, 0, -Fixed.piece_bottom_skin / 2000]) - center))
        else:
            add_mesh(asset, part.shape.label, part.shape, properties['center_mm'] if moving else (0, 0, 0))
            shape_attributes = dict(type='mesh', mesh=part.shape.label)
        collision = part.group in ('Playing_surface', 'King_envelope', 'Piece_skin')
        color = list(part.shape.color)
        if part.overlay and part.group != 'King_envelope':
            color[-1] = 0
        xml.SubElement(king if moving else world, 'geom', name=part.shape.label, **shape_attributes,
                       contype=str(int(collision)), conaffinity=str(int(collision)), rgba=numbers(color))
    xml.indent(scene)
    metadata = dict(king_mass_kg=properties['mass_g'] / 1000, king_com_from_magnet_bottom_m=center.tolist(),
                    king_inertia_kg_m2=properties['inertia_kg_m2'], physical_coils=len(array.identities),
                    physical_identities=array.identities,
                    board_top_m=(-Inputs.magnet_to_coil_distance + Fixed.potting_cover_thickness + Fixed.playing_surface_thickness) / 1000,
                    base_radius_m=properties['bottom_film_radius_m'],
                    collision_scope='Exact bottom-film cylinder against a local infinite plane at the CAD playing-surface height; convex exterior mesh for the king. Board edges and concave piece-to-piece contact remain unqualified.',
                    friction_scope='Single Coulomb coefficient from resting_friction_coefficient; separate static/kinetic friction and torsional/rolling resistance remain unqualified.',
                    damping_scope='Soft-contact numerical damping only. No artificial airborne joint damping or material vibration model.')
    return xml.tostring(scene, encoding='unicode'), metadata
