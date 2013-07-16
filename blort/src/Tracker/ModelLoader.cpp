#include <ros/console.h>
#include <blort/Tracker/ModelLoader.h>
#include <stdexcept>

using namespace Tracking;
using namespace TomGine;
using namespace std;

// *** PRIVATE ***

// Tests if property of file is available in list (data structure)
bool ModelLoader::propertyIsInList(PlyProperty* prop, PlyProperty* list, int n, int* index){

    for(int i=0; i<n; i++){
        if(equal_strings(prop->name, list[i].name)){
            *index = i;
            return true;
        }
    }
    return false;
}

// *** PUBLIC ***

ModelLoader::ModelLoader(){
}

ModelLoader::~ModelLoader(){

}

// read ply file
bool ModelLoader::LoadPly(TomGine::tgModel &model, const char* filename){
    PlyFile* plyfile;
    int i,j;
    int nelems;
    char **elist;
    int file_type;
    float version;
    char *elem_name;
    //PlyElement *elem_ptr;
    PlyProperty **plist;
    //PlyProperty* prop_ptr;
    int num_elems;
    int nprops;
    int index;
    char** obj_info;
    int num_obj_info;

    int num_vertices = 0;
    int num_faces = 0;
//    int num_edges;
    PlyVertex* plyvertexlist = 0;
    PlyFace* plyfacelist = 0;
    PlyEdge* plyedgelist = 0;
    vector<string> texFilenames;
    string strFilename(filename);


    // **********************************************
    // Read data from ply file

    // open file
    plyfile = ply_open_for_reading((char*)filename, &nelems, &elist, &file_type, &version);
    if(plyfile==0){
        char errmsg[128];
        sprintf(errmsg, "[ModelLoader::LoadPly] Error loading ply file %s\n", filename);
        throw std::runtime_error(errmsg);
    }
    // 	sprintf(model.m_modelname, "%s", filename);

    // Load texture files from obj_info (=texture-filename)
    obj_info = ply_get_obj_info(plyfile, &num_obj_info);
    if(num_obj_info < 1){
   	//printf("[ModelLoader::read] Warning no texture found in model %s\n", filename);
    }else if(num_obj_info >= 1){
        for(i=0; i<num_obj_info; i++){
            texFilenames.push_back("");
            texFilenames[i].append(filename, 0, strFilename.find_last_of("/")+1);
            texFilenames[i].append(obj_info[i]);
            // 			printf("[ModelLoader::read] obj_info: %s\n", texFilenames[i].c_str());
        }
    }

    // list of property information for a vertex
    PlyProperty vert_props[] = {
        {(char*)"x", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,x), 0, 0, 0, 0},
        {(char*)"y", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,y), 0, 0, 0, 0},
        {(char*)"z", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,z), 0, 0, 0, 0},
        {(char*)"nx", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,nx), 0, 0, 0, 0},
        {(char*)"ny", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,ny), 0, 0, 0, 0},
        {(char*)"nz", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,nz), 0, 0, 0, 0},
        {(char*)"s", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,s), 0, 0, 0, 0},
        {(char*)"t", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,t), 0, 0, 0, 0},
        {(char*)"red", PLY_UCHAR, PLY_UCHAR, offsetof(PlyVertex,r), 0, 0, 0, 0},
        {(char*)"green", PLY_UCHAR, PLY_UCHAR, offsetof(PlyVertex,g), 0, 0, 0, 0},
        {(char*)"blue", PLY_UCHAR, PLY_UCHAR, offsetof(PlyVertex,b), 0, 0, 0, 0}
    };

    // list of property information for a face
    PlyProperty face_props[] = { /* list of property information for a vertex */
        {(char*)"vertex_indices", PLY_USHORT, PLY_UINT, offsetof(PlyFace,v),
         1, PLY_USHORT, PLY_UINT, offsetof(PlyFace,nverts)},
};

    // list of property information for an edge
    PlyProperty edge_props[] = {
        {(char*)"start", PLY_USHORT, PLY_USHORT, offsetof(PlyEdge,start), 0, 0, 0, 0},
        {(char*)"end", PLY_USHORT, PLY_USHORT, offsetof(PlyEdge,end), 0, 0, 0, 0}
    };

    for(i=0; i<nelems; i++){
        // get description of element
        elem_name = elist[i];
        plist = ply_get_element_description (plyfile, elem_name, &num_elems, &nprops);

        // *** Read Vertices ***
        if (equal_strings ((char*)"vertex", elem_name)) {
            // allocate memory for vertices
            num_vertices = num_elems;
            plyvertexlist = (PlyVertex*)malloc(sizeof(PlyVertex) * num_vertices);

            // setup property specification for elements
            for(j=0; j<nprops; j++){
                if(propertyIsInList(plist[j], vert_props, 11, &index))
                    ply_get_property(plyfile, elem_name, &vert_props[index]);
            }

            // grab all vertex elements
            for(j=0; j<num_elems; j++){
                ply_get_element(plyfile, &plyvertexlist[j]);
            }

        }

        // *** Read Faces ***
        if (equal_strings ((char*)"face", elem_name)) {
            // allocate memory for faces
            num_faces = num_elems;
            plyfacelist = (PlyFace*)malloc(sizeof(PlyFace)*num_faces);

            // setup property specification for elements
            for(j=0; j<nprops && j<1; j++){
                ply_get_property(plyfile, elem_name, &face_props[j]);
            }

            // grab all face elements
            for(j=0; j<num_elems; j++){
                ply_get_element(plyfile, &plyfacelist[j]);
                //printf ("face: %d %d %d %d\n", m_facelist[j].v[0], m_facelist[j].v[1], m_facelist[j].v[2], m_facelist[j].v[3]);
            }
        }

        // *** Read Edges ***
        if (equal_strings ((char*)"edge", elem_name)) {
            // allocate memory for edges
            plyedgelist = (PlyEdge*)malloc(sizeof(PlyEdge)*num_elems);
            //num_edges = num_elems;

            // setup property specification for elements
            for(j=0; j<nprops && j<2; j++){
                ply_get_property(plyfile, elem_name, &edge_props[j]);
            }

            // grab all edge elements
            for(j=0; j<num_elems; j++){
                ply_get_element(plyfile, &plyedgelist[j]);
                //printf("edge: %d %d\n", m_edgelist[j].start, m_edgelist[j].end);
            }
        }
    }
    ply_close(plyfile);


    // **********************************************
    // Convert ply to model

    // Parse through vertex list
    for(i=0; i<num_vertices; i++){
        tgVertex v;
        v.pos.x = plyvertexlist[i].x;
        v.pos.y = plyvertexlist[i].y;
        v.pos.z = plyvertexlist[i].z;
        v.normal.x = plyvertexlist[i].nx;
        v.normal.y = plyvertexlist[i].ny;
        v.normal.z = plyvertexlist[i].nz;
        v.texCoord.x = plyvertexlist[i].s;
        v.texCoord.y = plyvertexlist[i].t;
        model.add(v);
    }

    // Parse through face list
    for(i=0; i<num_faces; i++){
        tgFace f;
        for(j=0; j<plyfacelist[i].nverts; j++){
            f.v.push_back(plyfacelist[i].v[j]);
        }
        model.add(f);
    }

    // 	for(i=0; i<num_edges; i++){
    // 		Model::Edge e;
    // 		e.start = plyedgelist[i].start;
    // 		e.end = plyedgelist[i].end;
    // 		model.m_edgelist.push_back(e);
    // 	}

    model.ComputeFaceNormals();

    // **********************************************
    // Clean up
    if(plyvertexlist) free(plyvertexlist);
    if(plyfacelist) free(plyfacelist);
    if(plyedgelist) free(plyedgelist);

    return true;
}

bool ModelLoader::LoadPly(TrackerModel &model, const char* filename){
    PlyFile* plyfile;
    int i,j;
    int nelems;
    char **elist;
    int file_type;
    float version;
    char *elem_name;
    //PlyElement *elem_ptr;
    PlyProperty **plist;
    //PlyProperty* prop_ptr;
    int num_elems;
    int nprops;
    int index;
    char** obj_info;
    int num_obj_info;

    int num_vertices = 0;
    int num_faces = 0;
//    int num_edges;
    int num_passes = 0;
    PlyVertex* plyvertexlist = 0;
    PlyFace* plyfacelist = 0;
    PlyEdge* plyedgelist = 0;
    PlyPass* plypasslist = 0;
    vector<string> texFilenames;
    string strFilename(filename);

    // **********************************************
    // Read data from ply file
    // open file
    plyfile = ply_open_for_reading((char*)filename, &nelems, &elist, &file_type, &version);
    if(plyfile==0){
        ROS_DEBUG("[ModelLoader::LoadPly] Failed to load ply file '%s'", filename);
        return false;
    }

    // 	sprintf(model.m_modelname, "%s", filename);

    // Load texture files from obj_info (=texture-filename)
    obj_info = ply_get_obj_info(plyfile, &num_obj_info);
    if(num_obj_info < 1){
   	//printf("[ModelLoader::read] Warning no texture found in model %s\n", filename);
    }else if(num_obj_info >= 1){
        for(i=0; i<num_obj_info; i++){
            texFilenames.push_back("");
            texFilenames[i].append(filename, 0, strFilename.find_last_of("/")+1);
            texFilenames[i].append(obj_info[i]);
            //  			printf("[ModelLoader::read] obj_info: %s\n", texFilenames[i].c_str());
        }
    }
    // list of property information for a vertex
    PlyProperty vert_props[] = {
        {(char*)"x", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,x), 0, 0, 0, 0},
        {(char*)"y", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,y), 0, 0, 0, 0},
        {(char*)"z", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,z), 0, 0, 0, 0},
        {(char*)"nx", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,nx), 0, 0, 0, 0},
        {(char*)"ny", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,ny), 0, 0, 0, 0},
        {(char*)"nz", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,nz), 0, 0, 0, 0},
        {(char*)"s", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,s), 0, 0, 0, 0},
        {(char*)"t", PLY_FLOAT, PLY_FLOAT, offsetof(PlyVertex,t), 0, 0, 0, 0},
        {(char*)"red", PLY_UCHAR, PLY_UCHAR, offsetof(PlyVertex,r), 0, 0, 0, 0},
        {(char*)"green", PLY_UCHAR, PLY_UCHAR, offsetof(PlyVertex,g), 0, 0, 0, 0},
        {(char*)"blue", PLY_UCHAR, PLY_UCHAR, offsetof(PlyVertex,b), 0, 0, 0, 0}
    };

    // list of property information for a face
    PlyProperty face_props[] = { /* list of property information for a vertex */
        {(char*)"vertex_indices", PLY_USHORT, PLY_UINT, offsetof(PlyFace,v),
         1, PLY_USHORT, PLY_UINT, offsetof(PlyFace,nverts)},
};

    // list of property information for an edge
    PlyProperty edge_props[] = {
        {(char*)"start", PLY_USHORT, PLY_USHORT, offsetof(PlyEdge,start), 0, 0, 0, 0},
        {(char*)"end", PLY_USHORT, PLY_USHORT, offsetof(PlyEdge,end), 0, 0, 0, 0}
    };

    PlyProperty pass_props[] = {
        {(char*)"face_indices", PLY_USHORT, PLY_UINT, offsetof(PlyPass,f),
         1, PLY_USHORT, PLY_UINT, offsetof(PlyPass,nfaces)},
{(char*)"m0", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m0), 0, 0, 0, 0},
{(char*)"m1", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m1), 0, 0, 0, 0},
{(char*)"m2", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m2), 0, 0, 0, 0},
{(char*)"m3", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m3), 0, 0, 0, 0},
{(char*)"m4", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m4), 0, 0, 0, 0},
{(char*)"m5", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m5), 0, 0, 0, 0},
{(char*)"m6", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m6), 0, 0, 0, 0},
{(char*)"m7", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m7), 0, 0, 0, 0},
{(char*)"m8", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m8), 0, 0, 0, 0},
{(char*)"m9", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m9), 0, 0, 0, 0},
{(char*)"m10", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m10), 0, 0, 0, 0},
{(char*)"m11", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m11), 0, 0, 0, 0},
{(char*)"m12", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m12), 0, 0, 0, 0},
{(char*)"m13", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m13), 0, 0, 0, 0},
{(char*)"m14", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m14), 0, 0, 0, 0},
{(char*)"m15", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,m15), 0, 0, 0, 0},
{(char*)"x", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,x), 0, 0, 0, 0},
{(char*)"y", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,y), 0, 0, 0, 0},
{(char*)"w", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,w), 0, 0, 0, 0},
{(char*)"h", PLY_FLOAT, PLY_FLOAT, offsetof(PlyPass,h), 0, 0, 0, 0},
{(char*)"tex_index", PLY_UCHAR, PLY_UCHAR, offsetof(PlyPass,tex), 0, 0, 0, 0}
    };

    for(i=0; i<nelems; i++){
        // get description of element
        elem_name = elist[i];
        plist = ply_get_element_description (plyfile, elem_name, &num_elems, &nprops);

        // *** Read Vertices ***
        if (equal_strings ((char*)"vertex", elem_name)) {
            // allocate memory for vertices
            num_vertices = num_elems;
            plyvertexlist = (PlyVertex*)malloc(sizeof(PlyVertex) * num_vertices);

            // setup property specification for elements
            for(j=0; j<nprops; j++){
                if(propertyIsInList(plist[j], vert_props, 11, &index))
                    ply_get_property(plyfile, elem_name, &vert_props[index]);
            }

            // grab all vertex elements
            for(j=0; j<num_elems; j++){
                ply_get_element(plyfile, &plyvertexlist[j]);
            }

        }

        // *** Read Faces ***
        if (equal_strings ((char*)"face", elem_name)) {
            // allocate memory for faces
            num_faces = num_elems;
            plyfacelist = (PlyFace*)malloc(sizeof(PlyFace)*num_faces);

            // setup property specification for elements
            for(j=0; j<nprops && j<1; j++){
                ply_get_property(plyfile, elem_name, &face_props[j]);
            }

            // grab all face elements
            for(j=0; j<num_elems; j++){
                ply_get_element(plyfile, &plyfacelist[j]);
                //printf ("face: %d %d %d %d\n", m_facelist[j].v[0], m_facelist[j].v[1], m_facelist[j].v[2], m_facelist[j].v[3]);
            }
        }

        // *** Read Edges ***
        if (equal_strings ((char*)"edge", elem_name)) {
            // allocate memory for edges
            plyedgelist = (PlyEdge*)malloc(sizeof(PlyEdge)*num_elems);
            //num_edges = num_elems;

            // setup property specification for elements
            for(j=0; j<nprops && j<2; j++){
                ply_get_property(plyfile, elem_name, &edge_props[j]);
            }

            // grab all edge elements
            for(j=0; j<num_elems; j++){
                ply_get_element(plyfile, &plyedgelist[j]);
                //printf("edge: %d %d\n", m_edgelist[j].start, m_edgelist[j].end);
            }
        }

        // *** Read Pass ***
        if (equal_strings ((char*)"pass", elem_name)) {
            num_passes = num_elems;
            plypasslist = (PlyPass*)malloc(sizeof(PlyPass)*num_passes);

            for(j=0; j<nprops; j++){
                ply_get_property(plyfile, elem_name, &pass_props[j]);
            }

            // grab all face elements
            for(j=0; j<num_passes; j++){
                ply_get_element(plyfile, &plypasslist[j]);
                //printf ("face: %d %d %d %d\n", m_facelist[j].v[0], m_facelist[j].v[1], m_facelist[j].v[2], m_facelist[j].v[3]);
            }
        }

    }
    ply_close(plyfile);

    // **********************************************
    // Convert ply to model

    // Parse through vertex list
    for(i=0; i<num_vertices; i++){
        tgVertex v;
        v.pos.x = plyvertexlist[i].x;
        v.pos.y = plyvertexlist[i].y;
        v.pos.z = plyvertexlist[i].z;
        v.normal.x = plyvertexlist[i].nx;
        v.normal.y = plyvertexlist[i].ny;
        v.normal.z = plyvertexlist[i].nz;
        v.texCoord.x = plyvertexlist[i].s;
        v.texCoord.y = plyvertexlist[i].t;
        model.add(v);
    }

    // Parse through face list
    for(i=0; i<num_faces; i++){
        tgFace f;
        for(j=0; j<plyfacelist[i].nverts; j++){
            f.v.push_back(plyfacelist[i].v[j]);
        }
        model.add(f);
    }

    // 	for(i=0; i<num_edges; i++){
    // 		Model::Edge e;
    // 		e.start = plyedgelist[i].start;
    // 		e.end = plyedgelist[i].end;
    // 		model.m_edgelist.push_back(e);
    // 	}

    for(i=0; i<num_passes; i++){
        TrackerModel::Pass* p = new TrackerModel::Pass;

        for(j=0; j<plypasslist[i].nfaces; j++){
            p->f.push_back(plypasslist[i].f[j]);
        }
	
        p->modelviewprojection.mat[0] = plypasslist[i].m0;
        p->modelviewprojection.mat[1] = plypasslist[i].m1;
        p->modelviewprojection.mat[2] = plypasslist[i].m2;
        p->modelviewprojection.mat[3] = plypasslist[i].m3;
        p->modelviewprojection.mat[4] = plypasslist[i].m4;
        p->modelviewprojection.mat[5] = plypasslist[i].m5;
        p->modelviewprojection.mat[6] = plypasslist[i].m6;
        p->modelviewprojection.mat[7] = plypasslist[i].m7;
        p->modelviewprojection.mat[8] = plypasslist[i].m8;
        p->modelviewprojection.mat[9] = plypasslist[i].m9;
        p->modelviewprojection.mat[10] = plypasslist[i].m10;
        p->modelviewprojection.mat[11] = plypasslist[i].m11;
        p->modelviewprojection.mat[12] = plypasslist[i].m12;
        p->modelviewprojection.mat[13] = plypasslist[i].m13;
        p->modelviewprojection.mat[14] = plypasslist[i].m14;
        p->modelviewprojection.mat[15] = plypasslist[i].m15;

        p->x = plypasslist[i].x;
        p->y = plypasslist[i].y;
        p->w = plypasslist[i].w;
        p->h = plypasslist[i].h;

        p->texture->load(texFilenames[i].c_str());

        model.m_passlist.push_back(p);
    }

    if(!model.m_passlist.empty())
        model.m_textured = true;

    model.ComputeFaceNormals();
    model.computeEdges();
    model.Update();

    // **********************************************
    // Clean up
    if(plyvertexlist) free(plyvertexlist);
    if(plyfacelist) free(plyfacelist);
    if(plyedgelist) free(plyedgelist);
    if(plypasslist) free(plypasslist);
    return true;
}

bool ModelLoader::SavePly(TomGine::tgModel &model, const char* filename){
    FILE* pFile;
    pFile = fopen(filename, "w");
    //fputs("[ModelLoader::SavePly(Model)] Warning not implemented, use ModelLoader::SavePly(TrackerModel)", pFile);
    ROS_DEBUG("[ModelLoader::SavePly(Model)] Warning not implemented, use ModelLoader::SavePly(TrackerModel)");

    fclose(pFile);
    return true;
}

bool ModelLoader::SavePly(TrackerModel &model, const char* name){
    ROS_INFO("Saving model to %s", name);
    FILE* pFile = 0;
    unsigned i,j;
    string filename(name);
    string texname;
    string texfilename;
    char number[512];

    filename.append(".ply");
    pFile = fopen(filename.c_str(), "w");

    if(!pFile){
        ROS_ERROR("[ModelLoader::SavePly] Can not create file '%s'! Path existing?", name);
        throw std::runtime_error("[ModelLoader::SavePly] Can not create file! Path existing?");
    }

    // ****************************************************
    // Header
    fputs("ply\n", pFile);
    fputs("format ascii 1.0\n", pFile);

    // Save filenames of textures (relative to ply file)
    for(i=0; i<model.m_passlist.size(); i++){
        texname = string("");
        texfilename = string("");
        size_t start = filename.find_last_of("/");
        if(start!=filename.npos){
            string modelname(name);
            texname.append(modelname.begin()+start+1, modelname.end());
            texfilename.append(modelname.begin(), modelname.begin()+start+1);
        }else{
            texname = string(name);
        }

        texname.append("-");
        sprintf(number, "%.5u", i);
        texname.append(number);
        texname.append(".jpg");
        texfilename.append(texname);
        fputs("obj_info ", pFile);
        fputs(texname.c_str(), pFile);
        fputs("\n", pFile);
        model.m_passlist[i]->texture->save(texfilename.c_str());
    }

    // Header of vertex
    sprintf(number, "%d", model.getVertexSize());
    fputs("element vertex ", pFile);
    fputs(number, pFile);
    fputs("\n", pFile);
    fputs("property float x\n", pFile);
    fputs("property float y\n", pFile);
    fputs("property float z\n", pFile);
    fputs("property float nx\n", pFile);
    fputs("property float ny\n", pFile);
    fputs("property float nz\n", pFile);

    // Header of face
    sprintf(number, "%d", model.getFaceSize());
    fputs("element face ", pFile);
    fputs(number, pFile);
    fputs("\n", pFile);
    fputs("property list uchar uint vertex_indices\n", pFile);


    // Header of pass
    if(!model.m_passlist.empty()){
        sprintf(number, "%d", (int)model.m_passlist.size());
        fputs("element pass ", pFile);
        fputs(number, pFile);
        fputs("\n", pFile);
        fputs("property list uchar uint face_indices\n", pFile);	// f
        fputs("property float m0\n", pFile);
        fputs("property float m1\n", pFile);
        fputs("property float m2\n", pFile);
        fputs("property float m3\n", pFile);
        fputs("property float m4\n", pFile);
        fputs("property float m5\n", pFile);
        fputs("property float m6\n", pFile);
        fputs("property float m7\n", pFile);
        fputs("property float m8\n", pFile);
        fputs("property float m9\n", pFile);
        fputs("property float m10\n", pFile);
        fputs("property float m11\n", pFile);
        fputs("property float m12\n", pFile);
        fputs("property float m13\n", pFile);
        fputs("property float m14\n", pFile);
        fputs("property float m15\n", pFile);
        fputs("property float x\n", pFile);
        fputs("property float y\n", pFile);
        fputs("property float w\n", pFile);
        fputs("property float h\n", pFile);
        fputs("property uchar tex_index\n", pFile);
    }

    fputs("end_header\n",pFile);

    // Data of vertex
    for(i=0; i<model.getVertexSize(); i++){
        tgVertex v = model.getVertex(i);
        sprintf(	number, "%f %f %f %f %f %f\n",
                        v.pos.x, v.pos.y, v.pos.z,
                        v.normal.x, v.normal.y, v.normal.z);
        fputs(number, pFile);
    }

    // Data of face
    for(i=0; i<model.getFaceSize(); i++){
        tgFace f = model.getFace(i);
        sprintf(number, "%d", (int)f.v.size());
        fputs(number, pFile);
        for(j=0; j<f.v.size(); j++){
            sprintf( number, " %d", f.v[j]);
            fputs(number, pFile);
        }
        fputs("\n", pFile);
    }

    // Data of pass
    for(i=0; i<model.m_passlist.size(); i++){
        TrackerModel::Pass* p = model.m_passlist[i];
        sprintf(number, "%d", (int)p->f.size());
        fputs(number, pFile);
        for(j=0; j<p->f.size(); j++){			// Facelist
            sprintf(number, " %d", p->f[j]);
            fputs(number, pFile);
        }
        for(j=0; j<16; j++){
            sprintf(number, " %f", p->modelviewprojection.mat[j]);
            fputs(number, pFile);
        }
        sprintf(number, " %f %f %f %f", p->x,  p->y,  p->w,  p->h);
        fputs(number, pFile);
        sprintf(number, " %u", i);
        fputs(number, pFile);
        fputs("\n", pFile);
    }

    fclose(pFile);

    printf("saved: %s\n", filename.c_str());

    return true;
}

