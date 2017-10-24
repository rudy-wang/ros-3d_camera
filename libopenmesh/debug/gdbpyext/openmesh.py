import gdb
import gdbpyext.common as gc
import gdbpyext.eigen as ge
import gdbpyext.stl as gs

def vprops(mesh):
    # get offset for vertex properites
    idx = int(mesh["data_vpph_"]["idx_"])
    # get datatype of vertex properites
    dataT = mesh["data_vpph_"].type.strip_typedefs().template_argument(0)
    # get property type
    propT = gdb.lookup_type("OpenMesh::PropertyT<%s>" % dataT)
    # get vertex property container
    vpc = mesh.cast(gdb.lookup_type("OpenMesh::BaseKernel"))["vprops_"]["properties_"]
    # get vertex properties
    prop = gs.xVecElement(vpc, idx).cast(propT.pointer()).dereference()["data_"]
    return gs.xVec(prop)

def vpoints(mesh,vh_list,prop = ""):
    prop_mesh = vprops(mesh)
    if prop == "":
        return [] # replace with default vertex points
    return [ ge.eig2numpy(prop_mesh[int(gc.py("%s.idx_" % vh))][prop]) for vh in vh_list ]
        
