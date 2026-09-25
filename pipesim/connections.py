"""Socket fit and shared-bore occupancy rules used by authoring and validation."""
from .document import DocumentError


def socket_blockers(assembly, connector, port, ignore=None):
    fitting=assembly.parts[connector]
    excluded={port,*fitting.ports[port].get('excludes',[])}
    return [j for j in assembly.joints if j['id']!=ignore and any(
        e['part']==connector and (e.get('port') in excluded or
        port in fitting.ports.get(e.get('port'),{}).get('excludes',[]))
        for e in (j['a'],j['b']))]


def socket_attachment(assembly, member, connector, port, end='start', insertion_mm=None, at_mm=None, ignore=None):
    if member not in assembly.parts or connector not in assembly.parts:
        raise DocumentError('Select an existing member and connector')
    tube=assembly.parts[member]; fitting=assembly.parts[connector]
    if tube.kind!='member': raise DocumentError('Select a pipe, dowel or extrusion to connect')
    socket=fitting.ports.get(port,{})
    if socket.get('type')!='socket': raise DocumentError('Target must be a socket')
    if socket_blockers(assembly,connector,port,ignore): raise DocumentError('This socket or its shared bore is occupied')
    section=tube.section
    profile='round' if section.get('type') in ('tube','round','circle') else section.get('profile',section.get('type'))
    if profile!=socket.get('profile','round') or abs(section.get('diameter_mm',0)-socket.get('diameter_mm',0))>.6:
        raise DocumentError('The tube size or profile does not match this socket')
    if socket.get('through'):
        station=float(tube.length/2 if at_mm is None else at_mm)
        half=socket.get('engagement_mm',0)/2
        if not half<=station<=tube.length-half:
            raise DocumentError(f'The pipe must span the socket: use a station from {half:g} to {tube.length-half:g} mm from pipe start')
        return {'part':member,'at_mm':station},0.
    if end not in ('start','end'): raise DocumentError('Choose the start or end of the pipe')
    if any(j['id']!=ignore and any(e['part']==member and e.get('end')==end for e in (j['a'],j['b'])) for j in assembly.joints):
        raise DocumentError('This pipe end is already occupied')
    depth=float(min(30,socket['engagement_mm']*.8) if insertion_mm is None else insertion_mm)
    if not socket.get('min_engagement_mm',0)<=depth<=socket['engagement_mm']:
        raise DocumentError(f'Insertion must be {socket.get("min_engagement_mm",0):g}–{socket["engagement_mm"]:g} mm. Use a through socket to position a fitting along a pipe.')
    return {'part':member,'end':end},depth
