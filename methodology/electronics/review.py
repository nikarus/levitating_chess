import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from xml.etree import ElementTree
from zipfile import ZIP_DEFLATED, ZipFile

import sexpdata

import model
from model import Inputs, Fixed
from methodology import prototype_coils, source_fingerprints


def load_footprint(footprint_library, identifier):
    import pcbnew

    library, name = identifier.split(':')
    root=Path(__file__).parent if library=='FlyingChess' else footprint_library
    filename = root / f'{library}.pretty' / f'{name}.kicad_mod'
    footprint = pcbnew.FootprintLoad(str(filename.parent), name)
    if footprint is None:
        raise ValueError(f'Missing footprint: {filename}')
    if identifier=='FlyingChess:WSLP1206_2m_5m9':
        validate_shunt_footprint(footprint)
    footprint.BuildCourtyardCaches()
    courtyard = footprint.GetCourtyard(pcbnew.F_CrtYd)
    if courtyard.OutlineCount() != 1 or courtyard.Area() <= 0:
        raise ValueError(f'Footprint requires a single closed front courtyard: {filename}')
    return footprint, filename


def validate_shunt_footprint(footprint):
    import pcbnew

    length,width,gap=Fixed.electronics_shunt_land_pattern
    pads=sorted(footprint.Pads(),key=lambda pad:pad.GetNumber())
    if [pad.GetNumber() for pad in pads]!=['1','2']:
        raise ValueError('Current shunt requires two reviewed solder pads')
    for pad,sign in zip(pads,(-1,1),strict=True):
        expected=(sign*(gap+length)/2,0.,length,width)
        actual=tuple(pcbnew.ToMM(value) for value in (*pad.GetPosition(),*pad.GetSize()))
        if any(abs(a-b)>Fixed.cad_geometry_tolerance for a,b in zip(actual,expected,strict=True)):
            raise ValueError('Current-shunt land pattern differs from manufacturer/model dimensions')
    shapes=[shape for shape in footprint.GraphicalItems() if shape.GetLayer()==pcbnew.F_Fab]
    if len(shapes)!=1:
        raise ValueError('Current shunt requires one nominal fabrication outline')
    outline=shapes[0].GetBoundingBox()
    actual=(pcbnew.ToMM(outline.GetWidth()),pcbnew.ToMM(outline.GetHeight()))
    stroke=pcbnew.ToMM(shapes[0].GetWidth())
    if any(abs(a-stroke-b)>Fixed.cad_geometry_tolerance for a,b in zip(actual,Fixed.electronics_shunt_body,strict=True)):
        raise ValueError('Current-shunt body outline differs from nominal model dimensions')
    courtyard=[shape for shape in footprint.GraphicalItems() if shape.GetLayer()==pcbnew.F_CrtYd]
    if len(courtyard)!=1:
        raise ValueError('Current shunt requires one rectangular courtyard')
    box=courtyard[0].GetBoundingBox()
    stroke=pcbnew.ToMM(courtyard[0].GetWidth())
    actual=(pcbnew.ToMM(box.GetWidth())-stroke,pcbnew.ToMM(box.GetHeight())-stroke)
    expected=(2*length+gap+2*Fixed.electronics_footprint_courtyard_margin,
        width+2*Fixed.electronics_footprint_courtyard_margin)
    if any(abs(a-b)>Fixed.cad_geometry_tolerance for a,b in zip(actual,expected,strict=True)):
        raise ValueError('Current-shunt courtyard differs from model clearance')


def validate_integrated_channel(netlist):
    connected = {}
    for net in netlist.findall('nets/net'):
        for node in net.findall('node'):
            key = (node.attrib['ref'], node.attrib['pin'])
            if key in connected:
                raise ValueError(f'Pin belongs to multiple nets: {key}')
            connected[key] = net.attrib['name'].removeprefix('/')
    driver_nets = {
        'GND': (1, 35, 36, 37, 38, 40, 44),
        'PGND_A_P': (20,), 'PGND_A_N': (13,), 'PGND_B_P': (3,), 'PGND_B_N': (10,),
        'V_MOTOR': (2, 11, 12, 21), 'VCP': (22,), 'CPH': (23,), 'CPL': (24,),
        'AWAKE': (25,), 'PWM_A_P': (26,), 'PWM_A_N': (27,),
        'PWM_B_P': (28,), 'PWM_B_N': (29,), 'ENABLE_A': (30, 31),
        'ENABLE_B': (32, 33), 'VREF_CURRENT': (34,), 'DVDD': (39, 42, 43), 'FAULT_N': (41,),
        'COIL_A_P': (17, 18, 19), 'COIL_A_N': (14, 15, 16),
        'COIL_B_P': (4, 5, 6), 'COIL_B_N': (7, 8, 9),
    }
    expected = {('U1', str(pin)): net for net, pins in driver_nets.items() for pin in pins}
    for channel, amplifier, shunts, bypass, resistor, capacitors in (
            ('A', 'U2', ('R5','R7'), 'C10','R9',('C12','C13')),
            ('B', 'U3', ('R6','R8'), 'C11','R10',('C14','C15'))):
        for pin, net in {
            '1': f'AMP_{channel}', '2': 'GND', '3': f'PGND_{channel}_N', '4': f'PGND_{channel}_P',
            '5': 'VREF_CURRENT', '6': 'V_ANALOG',
        }.items():
            expected[(amplifier, pin)] = net
        for shunt,leg in zip(shunts,('P','N'),strict=True):
            expected[(shunt, '1')] = f'PGND_{channel}_{leg}'
            expected[(shunt, '2')] = 'GND'
        expected[(resistor,'1')]=f'AMP_{channel}'
        expected[(resistor,'2')]=f'CURRENT_{channel}'
        for capacitor in capacitors:
            expected[(capacitor,'1')]=f'CURRENT_{channel}'
            expected[(capacitor,'2')]='GND'
        expected[(bypass, '1')] = 'V_ANALOG'
        expected[(bypass, '2')] = 'GND'
    for reference, nets in {
        'C1': ('V_MOTOR', 'PGND_B_P'), 'C2': ('V_MOTOR', 'PGND_B_N'),
        'C3': ('V_MOTOR', 'PGND_A_N'), 'C4': ('V_MOTOR', 'PGND_A_P'),
        'C5': ('V_MOTOR', 'PGND_A_P'), 'C6': ('VCP', 'V_MOTOR'), 'C7': ('CPH', 'CPL'),
        'C16': ('V_MOTOR','PGND_B_P'), 'C8': ('DVDD', 'GND'), 'C9': ('DVDD', 'GND'),
        'R1': ('FAULT_N', 'V_LOGIC'), 'R2': ('AWAKE', 'GND'),
        'R3': ('ENABLE_A', 'GND'), 'R4': ('ENABLE_B', 'GND'),
    }.items():
        expected.update({(reference, str(index)): net for index, net in enumerate(nets, start=1)})
    if connected != expected:
        differences = {str(pin): dict(expected=expected.get(pin), actual=connected.get(pin))
                       for pin in expected.keys() | connected.keys() if expected.get(pin) != connected.get(pin)}
        raise ValueError(f'Integrated channel connectivity differs from reviewed DDV circuit: {differences}')
    components = {part.attrib['ref']: part for part in netlist.findall('components/comp')}
    for reference, mpn in (('U1', Fixed.electronics_integrated_driver[0]),
                           ('U2', 'INA181A1IDBVR'), ('U3', 'INA181A1IDBVR')):
        fields = {field.attrib['name']: field.text for field in components[reference].findall('fields/field')}
        if fields['MPN'] != mpn:
            raise ValueError(f'Unreviewed active component: {reference}')
    return dict(passed=True, checked_pins=len(expected),
                scope='DDV pin map, separate PGND shunts, INA181 polarity, physical RC and local support; not complete circuit functionality')


def native_component_parameters(schematic):
    tree=sexpdata.loads(schematic.read_text())
    result={}
    for node in tree:
        if isinstance(node,list) and node and node[0]==sexpdata.Symbol('symbol'):
            fields={str(item[1]):str(item[2]) for item in node if isinstance(item,list) and item and item[0]==sexpdata.Symbol('property')}
            parameter=fields['ModelParameter']
            divisor=float(fields['ModelDivisor'])
            if divisor<=0:
                raise ValueError('Component model divisor must be positive')
            if parameter and float(fields['Value'])!=getattr(Fixed,parameter)/divisor:
                raise ValueError(f"Stale model-derived native component: {fields['Reference']}")
            result[fields['Reference']]=(parameter,divisor)
    return result


def validate_acquisition(netlist):
    expected={}
    adc={'ADC_SDI':(1,), 'ADC_RESET_N':(2,), 'ADC_REFIO':(5,), 'ADC_REFCAP':(7,),
         'GND':(3,4,6,8,10,11,13,15,17,19,20,22,24,26,28,29,31,32,33),
         'V_ANALOG':(9,30), 'V_LOGIC':(34,), 'ADC_ALARM':(35,), 'ADC_SDO':(36,),
         'ADC_SCLK':(37,), 'ADC_CS_N':(38,)}
    for channel,pin in enumerate((16,18,21,23,25,27,12,14)):
        adc[f'CURRENT_{channel}']=(pin,)
    expected.update({('U10',str(pin)):net for net,pins in adc.items() for pin in pins})
    expected.update({('U11',pin):net for pin,net in {'2':'V_ANALOG','4':'GND','5':'BIAS_NR','6':'VREF_CURRENT'}.items()})
    for reference,nets in {
        'C20':('V_ANALOG','GND'),'C21':('V_ANALOG','GND'),'C22':('V_ANALOG','GND'),
        'C23':('V_ANALOG','GND'),'C24':('V_LOGIC','GND'),'C25':('V_LOGIC','GND'),
        'C26':('ADC_REFIO','GND'),'C27':('ADC_REFCAP','GND'),'C28':('ADC_REFCAP','GND'),
        'C29':('V_ANALOG','GND'),'C30':('BIAS_CAP','GND'),'C31':('BIAS_NR','GND'),
        'R20':('VREF_CURRENT','BIAS_CAP'),'R21':('ADC_RESET_N','GND'),'R22':('ADC_CS_N','V_LOGIC'),
    }.items():
        expected.update({(reference,str(pin)):net for pin,net in enumerate(nets,1)})
    connected={}
    for net in netlist.findall('nets/net'):
        if net.attrib['name'].startswith('unconnected-'):
            continue
        for node in net.findall('node'):
            key=(node.attrib['ref'],node.attrib['pin'])
            if key in connected:
                raise ValueError(f'Pin belongs to multiple nets: {key}')
            connected[key]=net.attrib['name'].removeprefix('/')
    if connected!=expected:
        differences={str(pin):(expected.get(pin),connected.get(pin)) for pin in expected.keys()|connected.keys()
                     if expected.get(pin)!=connected.get(pin)}
        raise ValueError(f'Acquisition pin mapping or decoupling differs: {differences}')
    for reference,mpn in (('U10',Fixed.electronics_acquisition_sar[0]),('U11','REF5025IDR')):
        component=netlist.find(f"components/comp[@ref='{reference}']")
        if component.find("fields/field[@name='MPN']").text!=mpn:
            raise ValueError(f'Unreviewed acquisition part or accuracy grade: {reference}')
    return dict(passed=True,checked_pins=len(expected),
        scope='ADS8698 pin table, internal reference support and independent REF5025 bias; shared rails and logic remain external')



def validate_controller(netlist):
    gpio_pins=(77,78,79,80,1,2,3,4,6,7,8,9,11,12,13,14,16,17,18,19,20,21,22,23,
               25,26,27,28,36,37,38,39,40,42,43,44,45,46,47,48,49,52,53,54,55,56,57,58)
    expected={('U20',str(pin)):net for pin,net in zip(gpio_pins,Fixed.electronics_logic_gpio_nets,strict=True)}
    for net,pins in {
        'V_LOGIC':(5,15,24,29,41,50,60,76,59,64,68,69),'V_CORE':(10,32,51,65),
        'VREG_AVDD':(61,),'GND':(62,81),'VREG_LX':(63,), 'XIN':(30,), 'XOUT':(31,),
        'SWCLK':(33,), 'SWDIO':(34,), 'RUN':(35,),
    }.items():
        expected.update({('U20',str(pin)):net for pin in pins})
    passives={'L20':('V_CORE','VREG_LX'),'R30':('V_LOGIC','VREG_AVDD'),
        'R31':('XOUT','XTAL_OUT'),'R32':('RUN','V_LOGIC'),'R33':('SAFE_OK','GND'),
        'Y20':('XIN','GND','XTAL_OUT','GND')}
    capacitors=('V_LOGIC','V_CORE','VREG_AVDD','XIN','XTAL_OUT')+('V_LOGIC',)*11+('V_CORE',)*3
    passives.update({f'C{40+index}':(net,'GND') for index,net in enumerate(capacitors)})
    for reference,nets in passives.items():
        expected.update({(reference,str(pin)):net for pin,net in enumerate(nets,1)})
    connected={(node.attrib['ref'],node.attrib['pin']):net.attrib['name'].removeprefix('/')
        for net in netlist.findall('nets/net') if not net.attrib['name'].startswith('unconnected-')
        for node in net.findall('node')}
    if connected!=expected:
        differences={str(pin):(expected.get(pin),connected.get(pin)) for pin in expected.keys()|connected.keys()
                     if expected.get(pin)!=connected.get(pin)}
        raise ValueError(f'Controller pin assignment or support circuit differs: {differences}')
    for reference,mpn in (('U20',Fixed.electronics_local_controller),('L20','AOTA-B201610S3R3-101-T'),('Y20','ABM8-272-T3')):
        component=netlist.find(f"components/comp[@ref='{reference}']")
        if component.find("fields/field[@name='MPN']").text!=mpn:
            raise ValueError(f'Unreviewed controller part: {reference}')
    return dict(passed=True,checked_pins=len(expected),
        scope='RP2354B pin assignment, core supply, clock and bypass connectivity; firmware, protection, rails and PCB layout unqualified')


def validate_protection(netlist, reference_offset=0):
    expected = {
        'U30': {'1':'WATCHDOG_PULSE','2':'GND','3':'WATCHDOG_SET','4':'WATCHDOG_DIV','5':'V_LOGIC','6':'WATCHDOG_OK'},
        'U31': {'1':'WATCHDOG_OK','2':'GND','3':'FAULT_N','4':'CLEAR_N','5':'V_LOGIC','6':'POWER_FAULT_N'},
        'U32': {'1':'ARM_REQUEST','2':'V_LOGIC','4':'GND','5':'SAFE_OK','6':'CLEAR_N','7':'V_LOGIC','8':'V_LOGIC'},
        'R40': {'1':'WATCHDOG_SET','2':'GND'}, 'R41': {'1':'V_LOGIC','2':'WATCHDOG_DIV'},
        'R42': {'1':'WATCHDOG_DIV','2':'GND'}}
    for index in range(Fixed.electronics_acquisition_sar[2]//2):
        expected[f'U{33+index}'] = {'1':f'ENABLE_REQUEST_{2*index}','2':'SAFE_OK','7':f'ENABLE_{2*index}',
            '5':f'ENABLE_REQUEST_{2*index+1}','6':'SAFE_OK','3':f'ENABLE_{2*index+1}','4':'GND','8':'V_LOGIC'}
    for index, net in enumerate(('ARM_REQUEST','WATCHDOG_PULSE','POWER_FAULT_N','CLEAR_N',
            *(f'ENABLE_REQUEST_{i}' for i in range(Fixed.electronics_acquisition_sar[2])))):
        expected[f'R{43+index}'] = {'1':net,'2':'GND'}
    for index in range(7):
        expected[f'C{70+index}'] = {'1':'V_LOGIC','2':'GND'}
    offset = lambda reference: reference[0]+str(int(reference[1:])+reference_offset)
    expected = {(offset(ref),pin):net for ref,pins in expected.items() for pin,net in pins.items()}
    references = {ref for ref,pin in expected}
    connected = {(node.attrib['ref'],node.attrib['pin']):net.attrib['name'].split('/')[-1]
        for net in netlist.findall('nets/net') if not net.attrib['name'].startswith('unconnected-')
        for node in net.findall('node') if node.attrib['ref'] in references}
    if connected != expected:
        differences = {str(pin):(expected.get(pin),connected.get(pin)) for pin in expected.keys()|connected.keys()
            if expected.get(pin)!=connected.get(pin)}
        raise ValueError(f'Protection pin connectivity differs: {differences}')
    parts = {'U30':Fixed.electronics_watchdog_part,'U31':'SN74LVC1G11DBVR','U32':'SN74LVC1G74DCUR',
        **{f'U{33+i}':'SN74LVC2G08DCUR' for i in range(Fixed.electronics_acquisition_sar[2]//2)}}
    for ref,mpn in parts.items():
        part = netlist.find(f"components/comp[@ref='{offset(ref)}']")
        if part.find("fields/field[@name='MPN']").text != mpn:
            raise ValueError(f'Unreviewed protection part: {ref}')
    return dict(passed=True,checked_pins=len(expected),
        scope='Independent watchdog, asynchronous fault latch and separately gated coil enables; electrical timing, power supervision and arbitrary overcurrent protection remain unqualified')


def validate_control_group(netlist):
    components={part.attrib['ref']:part for part in netlist.findall('components/comp')}
    pins={(node.attrib['ref'],node.attrib['pin']):net.attrib['name'].removeprefix('/')
          for net in netlist.findall('nets/net') for node in net.findall('node')}
    by_mpn={}
    for reference,part in components.items():
        mpn=part.find("fields/field[@name='MPN']").text
        by_mpn.setdefault(mpn,[]).append(reference)
    controller,=by_mpn[Fixed.electronics_local_controller]
    adc,=by_mpn[Fixed.electronics_acquisition_sar[0]]
    drivers=sorted(by_mpn[Fixed.electronics_integrated_driver[0]])
    channels=Fixed.electronics_acquisition_sar[2]
    if len(drivers)*Fixed.electronics_integrated_driver[2]!=channels:
        raise ValueError('Control group does not provide exactly one independent bridge per ADC input')
    for reference,expected in ((controller,{'24':'V_LOGIC','81':'GND','25':'ADC_SCLK','26':'ADC_CS_N','27':'ADC_SDI','28':'ADC_SDO','36':'ADC_RESET_N','37':'ADC_ALARM'}),
            (adc,{'1':'ADC_SDI','2':'ADC_RESET_N','34':'V_LOGIC','35':'ADC_ALARM','36':'ADC_SDO','37':'ADC_SCLK','38':'ADC_CS_N'})):
        if any(pins[(reference,pin)]!=net for pin,net in expected.items()):
            raise ValueError('Shared ADC/control interface is not connected to the reviewed pins')
    pwm_pins=(77,78,79,80,1,2,3,4,6,7,8,9,11,12,13,14)
    adc_pins=(16,18,21,23,25,27,12,14)
    for channel in range(channels):
        driver=drivers[channel//2]
        first_pwm=26+2*(channel%2)
        for offset,leg in enumerate(('P','N')):
            net=f'PWM_{channel}_{leg}'
            if pins[(driver,str(first_pwm+offset))]!=net or pins[(controller,str(pwm_pins[2*channel+offset]))]!=net:
                raise ValueError('Independent PWM path is crossed or disconnected')
        if pins[(adc,str(adc_pins[channel]))]!=f'CURRENT_{channel}':
            raise ValueError('Current feedback channel mapping differs')
        sheet=components[driver].find('sheetpath').attrib['names']
        current_resistor=next(ref for ref,part in components.items()
            if part.find('sheetpath').attrib['names']==sheet and
            part.find("fields/field[@name='ModelParameter']").text=='electronics_acquisition_filter_resistance'
            and pins[(ref,'2')]==f'CURRENT_{channel}')
        if not pins[(current_resistor,'1')].endswith(f"AMP_{'AB'[channel%2]}"):
            raise ValueError('Current feedback is taken from another bridge amplifier')
        enable=f'ENABLE_{channel}'
        if any(pins[(driver,str(pin))]!=enable for pin in (30+2*(channel%2),31+2*(channel%2))):
            raise ValueError('Driver enable halves must share their independent protection output')
    validate_protection(netlist,reference_offset=700)
    for pin,net in (('45','WATCHDOG_PULSE'),('46','SAFE_OK'),('47','ARM_REQUEST'),('58','POWER_FAULT_N')):
        if pins[(controller,pin)]!=net:
            raise ValueError('Controller and protection handshake are disconnected')
    for channel in range(channels):
        if pins[(controller,str(16+channel))]!=f'ENABLE_REQUEST_{channel}':
            raise ValueError('Independent enable request is crossed or disconnected')
    return dict(passed=True,checked_channels=channels,component_count=len(components),
        scope='Composed PWM/current and independently gated enable paths; full power/protection timing and current limiting incomplete')


def export_review(schematic, footprint_library, executable, output, kind='channel'):
    import pcbnew

    if output.exists():
        raise ValueError('Preserve existing schematic review evidence')
    output.mkdir(parents=True)
    source_schematics=[schematic]
    if kind=='group':
        source_schematics += [schematic.parent/f'{name}.kicad_sch' for name in ('channel','acquisition','controller','protection')]
    for source_schematic in source_schematics:
        native_component_parameters(source_schematic)
    commands = [
        ['sch', 'export', 'netlist', '--format', 'kicadxml', '-o', str(output / 'netlist.xml')],
        ['sch', 'export', 'pdf', '-o', str(output / f'{schematic.stem}.pdf')],
        ['sch', 'export', 'bom', '--fields', 'Reference,Value,MPN,Footprint,${QUANTITY},ModelParameter',
         '--labels', 'References,Value,MPN,Footprint,Quantity,ModelParameter',
         '--group-by', 'MPN,Value,Footprint,ModelParameter', '-o', str(output / 'bom.csv')],
        ['sch', 'erc', '--format', 'json', '-o', str(output / 'erc.json')],
    ]
    logs = []
    for command in commands:
        result = subprocess.run([executable, *command, str(schematic)], capture_output=True, text=True, check=True)
        logs.append(dict(command=command, stdout=result.stdout, stderr=result.stderr))
    (output / 'export_log.json').write_text(json.dumps(logs, indent=Fixed.report_precision) + '\n')
    netlist = ElementTree.parse(output / 'netlist.xml').getroot()
    connectivity = {'channel':validate_integrated_channel,'acquisition':validate_acquisition,'controller':validate_controller,'group':validate_control_group,'protection':validate_protection}[kind](netlist)
    components = []
    footprint_paths = set()
    channel_count = 0
    for component in netlist.findall('components/comp'):
        fields = {field.attrib['name']: field.text for field in component.findall('fields/field')}
        if not fields.get('MPN'):
            raise ValueError(f"Missing part identity: {component.attrib['ref']}")
        if fields.get('CoilChannels') and (kind!='group' or fields['MPN']==Fixed.electronics_integrated_driver[0]):
            channel_count += int(fields['CoilChannels'])
        parameter = fields.get('ModelParameter')
        value = component.findtext('value')
        divisor=float(fields['ModelDivisor'])
        if parameter and float(value) != getattr(Fixed, parameter)/divisor:
            raise ValueError(f"Stale model-derived component value: {component.attrib['ref']}")
        footprint, filename = load_footprint(footprint_library, component.findtext('footprint'))
        footprint_paths.add(filename)
        courtyard = footprint.GetCourtyard(pcbnew.F_CrtYd)
        components.append(dict(reference=component.attrib['ref'], mpn=fields['MPN'], value=value,
            model_parameter=parameter, model_divisor=divisor, footprint=component.findtext('footprint'),
            courtyard_area_mm2=courtyard.Area() / pcbnew.FromMM(1)**2))
    expected_count=Fixed.electronics_integrated_driver[2] if kind=='channel' else Fixed.electronics_acquisition_sar[2]
    if channel_count != expected_count:
        raise ValueError('Schematic channel count differs from the reviewed integrated driver')
    _, coil, _ = prototype_coils.prototype_geometry(Inputs.cad_coil_layout)
    pitch_area = (coil.outer_width + Inputs.prototype_coil_spacing) * (coil.outer_length + Inputs.prototype_coil_spacing)
    courtyard_area = sum(part['courtyard_area_mm2'] for part in components)
    erc = json.loads((output / 'erc.json').read_text())
    violations = [violation for sheet in erc['sheets'] for violation in sheet['violations']]
    report = dict(status=f'draft_{kind}_incomplete_no_manufacturing_release',
        source_sha256=source_fingerprints(__file__, model.__file__, *source_schematics, schematic.parent / 'FlyingChess.kicad_sym',
            schematic.with_suffix('.kicad_pro'), schematic.parent / 'fp-lib-table', schematic.parent / 'sym-lib-table',
            prototype_coils.__file__),
        footprint_sha256={str(Path(path.parent.name)/path.name): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(footprint_paths)},
        component_count=len(components), coil_channel_count=channel_count, components=components,
        connectivity=connectivity,
        placement_bound=dict(courtyard_sum_mm2=courtyard_area,
            courtyard_per_channel_mm2=courtyard_area / channel_count, coil_pitch_area_mm2=pitch_area,
            double_sided_area_per_coil_mm2=2 * pitch_area,
            minimum_area_ratio_to_double_sided=courtyard_area / (channel_count * 2 * pitch_area),
            fits_even_before_routing=courtyard_area <= channel_count * 2 * pitch_area),
        erc=dict(errors=sum(row['severity'] == 'error' for row in violations),
            warnings=sum(row['severity'] == 'warning' for row in violations),
            passed=not violations),
        complete_hardware_baseline=False,
        limitations=[
            'Courtyard sum is an optimistic placement lower bound for this candidate and these footprints; no routing, shared circuit or thermal-spreading area is included.',
            'Both PCB faces are credited as fully usable, despite winding terminations and mechanical constraints.',
            'This is an integrated dual-channel circuit; the earlier placement predates its complete local support and must not be treated as placement of this netlist.',
            'Shared ADC/reference have a separate native sheet; power generation/regeneration, independent overcurrent shutdown, control and interconnect are not yet composed into a complete circuit.',
            'External named nets are interface contracts, not completed shared circuitry. No power flags or ERC exclusions hide missing sources.',
            'DDV pin mapping is checked; the library footprint remains provisional pending manufacturer land-pattern qualification.',
            'Internal current chopping is disabled for external precision regulation. Internal short-circuit protection does not establish the coil-safe current limit.',
            'No compatible driver SPICE model has been established. Connectivity is not functional simulation.',
            'INA181 separate-return sensing and output RC match the diagnostic topology. Complete acquisition, calibration and temperature qualification remain open.',
            'No component price, production BOM, thermal pass or hardware equivalence to the averaged simulator is claimed.'])
    if kind=='acquisition':
        report['limitations']=[
            'Native shared acquisition and bias sheet serves the configured ADC channel group; it is not yet composed with repeated channel sheets or selected logic and supplies.',
            'Pin connectivity and nominal passives are checked. Datasheet electrical limits, DC-bias capacitance, reference loading/noise, startup, calibration and PCB routing still require verification.',
            'Unconnected reference DNC/NC and unused TEMP pins are deliberate; missing power and digital drivers remain visible in ERC without power flags or exclusions.',
            'Courtyard sum amortized per served coil is only an optimistic placement-area contribution; this sheet does not contain the coil driver channels.',
            'The ADC and reference remain costed technical candidates, not a production BOM or complete functional simulation.']
    if kind in ('controller','group','protection'):
        report['limitations']=[
            'RP2354B is a candidate. Pin connectivity does not qualify real-time execution, GPIO timing, DMA service, PWM synchronization or watchdog shutdown.',
            'Core regulator parts and inductor polarity follow Raspberry Pi RP2350B Minimal R4. Exact critical placement/routing and effective capacitance must also follow the reference before acceptance.',
            'Three core, eight IOVDD, ADC, USB/OTP and QSPI supply bypass capacitors are included; stacked flash needs no external memory chip.',
            'Programming, host, Hall, temperature and fault nets are external interfaces. Independent enable gating is on the protection sheet; shared power supervision and rails remain external interfaces.',
            'No power flags or ERC exclusions hide missing rails or external circuitry. This is not a complete module or manufacturing release.',
            'Courtyard area is only a lower bound; crystal/regulator layout clearances, interfaces, routing and thermal paths require additional area.']
    if kind=='protection':
        report['limitations']=[
            'Native independent missing-pulse watchdog, asynchronous fault latch and per-coil enable gates; no autonomous current-limit comparator or complete supply supervisor is included.',
            'POWER_FAULT_N is externally driven by the unfinished rail-supervision circuit. Its pulldown keeps the group disabled until that circuit reports healthy power.',
            'Watchdog pulses must follow successfully validated sequential control commits. Hardware does not decode channel numbers or detect arbitrary corrupted PWM values.',
            'All propagation bounds depend on rail, temperature, load and input-edge conditions. Driver EN delay uses its specified maximum; timer output/trigger and complete shutdown timing remain unqualified.',
            'Cold startup, pad-level logic, independent rail supervision, layout and thermal coupling remain to be verified. No power flags or ERC exclusions hide missing interfaces.']
    with ZipFile(output / 'sources.zip', 'x', compression=ZIP_DEFLATED) as archive:
        for name in report['source_sha256']:
            archive.write(name, name)
        if kind in ('controller','group'):
            license_path=schematic.parent/'FlyingChess.pretty/RaspberryPi-LICENSE.txt'
            archive.write(license_path,str(license_path))
        for path in sorted(footprint_paths):
            archive.write(path, str(Path('kicad_footprints') / path.parent.name / path.name))
    (output / 'summary.json').write_text(json.dumps(report, indent=Fixed.report_precision) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Export a KiCad channel review and footprint-area lower bound.')
    parser.add_argument('--schematic', type=Path, required=True)
    parser.add_argument('--footprints', type=Path, required=True)
    parser.add_argument('--kicad-cli', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--kind', choices=('channel','acquisition','controller','group','protection'), default='channel')
    arguments = parser.parse_args()
    export_review(arguments.schematic, arguments.footprints, arguments.kicad_cli, arguments.output, arguments.kind)
