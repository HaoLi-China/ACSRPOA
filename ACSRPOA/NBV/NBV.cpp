#include "NBV.h"

CMesh original;
CMesh sample;
CMesh field_points;
CMesh tentative_mesh;
CMesh iso_points;

CMesh view_grid_points;
CMesh nbv_candidates;
vector<ScanCandidate>  scan_candidates;
Point3f                whole_space_box_max;
Point3f                whole_space_box_min;
Box3f                  whole_space_box;

double grid_step_size = (camera_far_dist_virtual * 2.0 + 1.0) / (grid_resolution - 1);
int x_max;
int y_max;
int z_max;

void loadMesh(char *fileName, CMesh &mesh)
{
  int mask= tri::io::Mask::IOM_VERTCOORD + tri::io::Mask::IOM_VERTNORMAL; 
  //+ tri::io::Mask::IOM_ALL + tri::io::Mask::IOM_FACEINDEX;

  int err = tri::io::Importer<CMesh>::Open(mesh, fileName, mask);  
  if(err) {
    cout << "Failed reading mesh: " << err << "\n";
    return;
  }  

  CMesh::VertexIterator vi;
  int idx = 0;
  for(vi = mesh.vert.begin(); vi != mesh.vert.end(); ++vi)
  {
    vi->m_index = idx++;
    mesh.bbox.Add(vi->P());
  }
  mesh.vn = mesh.vert.size();

  cout << "points loaded\n";
}

void saveMesh(char *fileName, CMesh &target)
{
  int mask= tri::io::Mask::IOM_VERTNORMAL;
  tri::io::ExporterPLY<CMesh>::Save(target, fileName, mask, false);
}
 
void clearCMesh(CMesh &mesh)
{
  mesh.face.clear();
  mesh.fn = 0;
  mesh.vert.clear();//TOTO:这里应该用swap清空内存
  mesh.vn = 0;
  mesh.bbox = Box3f();
}

void runPoissonExe()
{
  char mycmd[100];
  sprintf(mycmd, "PoissonRecon.exe --in poisson_in.ply --out poisson_out.ply --voxel poisson_field.raw --depth %d --pointWeight 0", 7);
  system(mycmd); 
}

void extractPoissonConfidence()
{
  FILE *fp = fopen("poisson_field.raw", "rb");
  if (fp == NULL) {
    perror("Open file poisson_field.raw");
    exit(1);
  }

  //int res = 1<<Par.Depth;
  //int read_size = res * res * res;
  int res;
  fread(&res, sizeof(int), 1, fp);

  float tree_scale;
  Point3f center_p;
  fread(&tree_scale, sizeof(float), 1, fp);
  fread(&center_p[0], sizeof(float), 1, fp);
  fread(&center_p[1], sizeof(float), 1, fp);
  fread(&center_p[2], sizeof(float), 1, fp);

  cout << res << " | " << tree_scale << " | " << center_p[0] << ", " << center_p[1] << ", " << center_p[2] << endl;

  int read_size = res * res * res;
  float *buf = new float[read_size];
  fread(buf, sizeof(float), read_size, fp);


  float space = tree_scale * (1.0 / res);
  int index = 0;
  field_points.vert.clear();
  int res2 = res * res;
  for (int i = 0; i < res; i++)
  {
    for (int j = 0; j < res; j++)
    {
      for (int k = 0; k < res; k++)
      {
        Point3f p(i * space, j * space, k * space);
        CVertex new_v;
        new_v.is_field_grid = true;
        new_v.P() = p + center_p;
        new_v.m_index = index;
        new_v.eigen_confidence = buf[i + j * res + k * res2];          
        index++;
        field_points.vert.push_back(new_v);
        field_points.bbox.Add(new_v.P());
      }
    }
  }

  field_points.vn = field_points.vert.size();
  cout << "field point size:  " << field_points.vn << endl;
  cout << "resolution:  " << res << endl;
}

void samplePointsFromMesh(CMesh &mesh, CMesh *points)
{
  mesh.bbox.SetNull();
  for (int i = 0; i < mesh.vert.size(); i++)
  {
    mesh.bbox.Add(mesh.vert[i]);
  }
  mesh.vn = mesh.vert.size();
  mesh.fn = mesh.face.size();
  vcg::tri::UpdateNormals<CMesh>::PerVertex(mesh);

  float radius = 0;
  int sampleNum = 1000; //para->getDouble("Poisson Disk Sample Number");
  if (sampleNum <= 100)
  {
    sampleNum = 100;
  }
  radius = tri::SurfaceSampling<CMesh,BaseSampler>::ComputePoissonDiskRadius(mesh, sampleNum);
  // first of all generate montecarlo samples for fast lookup
  CMesh *presampledMesh=&(mesh);
  CMesh MontecarloMesh; // this mesh is used only if we need real poisson sampling (and therefore we need to choose points different from the starting mesh vertices)


  BaseSampler sampler(&MontecarloMesh);
  sampler.qualitySampling =true;
  tri::SurfaceSampling<CMesh,BaseSampler>::Montecarlo(mesh, sampler, sampleNum*20);
  MontecarloMesh.bbox = mesh.bbox; // we want the same bounding box
  presampledMesh=&MontecarloMesh;


  BaseSampler mps(points);
  tri::SurfaceSampling<CMesh,BaseSampler>::PoissonDiskParam pp;
  tri::SurfaceSampling<CMesh,BaseSampler>::PoissonDisk(mesh, mps, *presampledMesh, radius,pp);
}

void extractIsoPoints()
{
  int mask= tri::io::Mask::IOM_VERTNORMAL ;
  int err = tri::io::Importer<CMesh>::Open(tentative_mesh, "poisson_out.ply", mask);  
  if(err) 
  {
    cout << "Failed reading mesh: " << err << "\n";
    return;
  }  

  if (tentative_mesh.vert.empty())
  {
    cout << "tentative mesh empty" << endl;
    return;
  }

  iso_points.vert.clear();
  samplePointsFromMesh(tentative_mesh, &iso_points);

  for (int i = 0; i < iso_points.vert.size(); i++)
  {
    CVertex& v = iso_points.vert[i];
    v.is_iso = true;
    v.m_index = i; 
    v.eigen_confidence = 0;
    v.N().Normalize();
    v.recompute_m_render();
  }
  iso_points.vn = iso_points.vert.size();
  std::cout<<"iso points num: " <<iso_points.vert.size() <<endl;
  saveMesh("Data/nbv_output/iso_points.ply", iso_points);
}

void computeAnnNeigbhors(vector<CVertex> &datapts, vector<CVertex> &querypts, int knn, bool need_self_included = false)
{
  int numKnn = knn + 1;

  vector<CVertex>::iterator vi_temp;
  for(vi_temp = datapts.begin(); vi_temp != datapts.end(); ++vi_temp)
    vi_temp->neighbors.clear();

  //if (querypts.size() <= numKnn+2)
  //{
  //  vector<CVertex>::iterator vi;
  //  for(vi = datapts.begin(); vi != datapts.end(); ++vi)
  //      vi->neighbors.clear();

  //  return;
  //}

  int					    nPts;			 // actual number of data points
  ANNpointArray		dataPts;	 // data points
  ANNpoint			  queryPt;	 // query point
  ANNidxArray			nnIdx;		 // near neighbor indices
  ANNdistArray		dists;		 // near neighbor distances
  ANNkd_tree*			kdTree;		 // search structure
  int			k				= numKnn;			      // number of nearest neighbors
  int			dim			= 3;			          // dimension
  double	eps			= 0;			          // error bound
  int			maxPts	= numKnn + 3000000;	// maximum number of data points

  if (datapts.size() >= maxPts)
  {
    cout << "Too many data" << endl;
    return;
  }
  queryPt = annAllocPt(dim);					// allocate query point
  dataPts = annAllocPts(maxPts, dim);	// allocate data points
  nnIdx   = new ANNidx[k];						// allocate near neigh indices
  dists   = new ANNdist[k];						// allocate near neighbor dists

  vector<CVertex>::iterator vi;
  int index = 0;
  for(vi = datapts.begin(); vi != datapts.end(); ++vi)
  {
    for(int j = 0; j < 3; j++)
      dataPts[index][j] = double(vi->P()[j]); 

    index++;
  }
  nPts = datapts.size();	 // read data points
  kdTree = new ANNkd_tree( // build search structure
    dataPts,					     // the data points
    nPts,						       // number of points
    dim);						       // dimension of space
  for (vi = querypts.begin(); vi != querypts.end(); ++vi) 
  {
    vi->neighbors.clear();
    for (int j = 0; j < 3; j++) 
      queryPt[j] = vi->P()[j];

    kdTree->annkSearch( // search
      queryPt,					// query point
      k,								// number of near neighbors
      nnIdx,						// nearest neighbors (returned)
      dists,						// distance (returned)
      eps);							// error bound

    for (int k = 1; k < numKnn; k++)
      vi->neighbors.push_back(nnIdx[k]);
  }
  delete [] nnIdx; // clean things up
  delete [] dists;
  delete kdTree;
  annClose();			 // done with ANN
}


void find_original_neighbors(CGrid::iterator starta, CGrid::iterator enda, 
  CGrid::iterator startb, CGrid::iterator endb, double radius) 
{	
  double radius2 = radius*radius;
  double iradius16 = -4/radius2;
  //const double PI = 3.1415926;

  for(CGrid::iterator dest = starta; dest != enda; dest++) 
  {
    CVertex &v = *(*dest);

    Point3f &p = v.P();
    for(CGrid::iterator origin = startb; origin != endb; origin++)
    {
      CVertex &t = *(*origin);

      Point3f &q = t.P();
      Point3f diff = p-q;

      double dist2 = diff.SquaredNorm();

      if(dist2 < radius2) 
      {                          
        v.original_neighbors.push_back((*origin)->m_index);
      }
    }
  }
}

// get neighbors
void self_neighbors(CGrid::iterator start, CGrid::iterator end, double radius)
{
  double radius2 = radius*radius;
  for(CGrid::iterator dest = start; dest != end; dest++)
  {
    CVertex &v = *(*dest);
    Point3f &p = v.P();


    for(CGrid::iterator origin = dest+1; origin != end; origin++)
    {
      CVertex &t = *(*origin);
      Point3f &q = t.P();
      Point3f diff = p-q;
      double dist2 = diff.SquaredNorm();
      if(dist2 < radius2) 
      {   
        v.neighbors.push_back((*origin)->m_index);
        t.neighbors.push_back((*dest)->m_index);
      }
    }
  }
}

void other_neighbors(CGrid::iterator starta, CGrid::iterator enda, 
  CGrid::iterator startb, CGrid::iterator endb, double radius)
{
  double radius2 = radius*radius;
  for(CGrid::iterator dest = starta; dest != enda; dest++)
  {
    CVertex &v = *(*dest);
    Point3f &p = v.P();

    for(CGrid::iterator origin = startb; origin != endb; origin++)
    {
      CVertex &t = *(*origin);
      Point3f &q = t.P();
      Point3f diff = p-q;
      double dist2 = diff.SquaredNorm();
      if(dist2 < radius2) 
      {   
        v.neighbors.push_back((*origin)->m_index);
        t.neighbors.push_back((*dest)->m_index);
      }
    }
  }
}

//mesh0: goal_set; mesh1: search_set
void computeBallNeighbors(CMesh* mesh0, CMesh* mesh1, double radius, vcg::Box3f& box)
{
  if (radius < 0.0001)
  {
    cout << "too small grid!!" << endl; 
    return;
  }
  //mesh1 should be original

  //cout << "compute_Bll_Neighbors" << endl;
  //cout << "radius: " << radius << endl;

  CGrid samples_grid;
  samples_grid.init(mesh0->vert, box, radius);
  //cout << "finished init" << endl;

  if (mesh1 != NULL)
  {
    for (int i = 0; i < mesh0->vn; i++)
    {
      mesh0->vert[i].original_neighbors.clear();
    }

    CGrid original_grid;
    original_grid.init(mesh1->vert, box, radius); // This can be speed up
    samples_grid.sample(original_grid, find_original_neighbors);
  }
  else
  {
    for (int i = 0; i < mesh0->vn; i++)
    {
      mesh0->vert[i].neighbors.clear();
    }

    samples_grid.iterate(self_neighbors, other_neighbors);
  }

}


double computeEulerDist(const Point3f& p1, const Point3f& p2)
{
  double dist2 = (p1-p2).SquaredNorm();
  if (dist2 < 1e-8 || dist2 > 1e8){
    return 0;
  }

  return sqrt(dist2);
}

double computeEulerDistSquare(Point3f& p1, Point3f& p2)
{
  return (p1-p2).SquaredNorm();
}

double getAbsMax(double x, double y, double z)
{
  return std::max(abs(x), std::max(abs(y), abs(z)));
}

void setGridUnHit(vector<int>& hit_grids_idx)
{
  vector<int>::iterator it;
  for (it = hit_grids_idx.begin(); it != hit_grids_idx.end(); ++it)
    view_grid_points.vert[*it].is_ray_hit = false;
}

bool isPointInBoundingBox(Point3f &v0, CMesh *mesh, double delta)
{
  if (NULL == mesh)
  {
    cout << "Empty Mesh When isPointInBoundingBox" << endl;
    return false; 
  }

  Point3f bmin = mesh->bbox.min - Point3f(delta, delta, delta);
  Point3f bmax = mesh->bbox.max + Point3f(delta, delta, delta);

  if ( v0[0] >= bmin[0] && v0[1] >= bmin[1] && v0[2] >= bmin[2]
  && v0[0] <= bmax[0] && v0[1] <= bmax[1] && v0[2] <= bmin[2])
    return true;
  else
    return false;
}


void deleteIgnore(CMesh* mesh)
{
  vector<CVertex> temp_vert;
  for (int i = 0; i < mesh->vert.size(); i++)
  {
    CVertex& v = mesh->vert[i];
    if (!v.is_ignore)
    {
      temp_vert.push_back(v);
    }
  }

  mesh->vert.clear();
  for (int i = 0; i < temp_vert.size(); i++)
  {
    CVertex& v = temp_vert[i];
    v.m_index = i;
    mesh->vert.push_back(v);
    mesh->bbox.Add(v.P());
  }
  mesh->vn = mesh->vert.size();
}

void normalizeROSA_Mesh(CMesh& mesh)
{
  if (mesh.vert.empty()) return;

  //get the normalize length
  float max_x = abs((mesh.bbox.min - mesh.bbox.max).X());
  float max_y = abs((mesh.bbox.min - mesh.bbox.max).Y());
  float max_z = abs((mesh.bbox.min - mesh.bbox.max).Z());
  float max_length = std::max(max_x, std::max(max_y, max_z));
  cout<<"max length: "<<max_length <<endl;

  mesh.bbox.SetNull();
  //float max_length = global_paraMgr.data.getDouble("Max Normalize Length");

  Box3f box_temp;
  for(int i = 0; i < mesh.vert.size(); i++)
  {
    Point3f &p = mesh.vert[i].P();
    p /= max_length;

    mesh.vert[i].N().Normalize(); 
    box_temp.Add(p);
  }

  Point3f mid_point = (box_temp.min + box_temp.max) / 2.0;

  for(int i = 0; i < mesh.vert.size(); i++)
  {
    Point3f& p = mesh.vert[i].P();
    p -= mid_point;
    mesh.bbox.Add(p);
  }
  
}

void normalizeConfidence(vector<CVertex>& vertexes, float delta)
{
  float min_confidence = INT_MAX;
  float max_confidence = INT_MIN;
  for (int i = 0; i < vertexes.size(); i++)
  {
    CVertex& v = vertexes[i];
    min_confidence = (std::min)(min_confidence, v.eigen_confidence);
    max_confidence = (std::max)(max_confidence, v.eigen_confidence);
  }

  cout<<max_confidence <<" " <<min_confidence <<endl;
  float space = max_confidence - min_confidence;
  
  for (int i = 0; i < vertexes.size(); i++)
  {
    CVertex& v = vertexes[i];
    v.eigen_confidence = (v.eigen_confidence - min_confidence) / space;
    v.eigen_confidence += delta;
  }
}

void computeHoleConfidence()
{
  assert(!original.vert.empty());
  assert(!iso_points.vert.empty());
  cout<<"compute hole confidence" <<endl;

 computeAnnNeigbhors(original.vert, iso_points.vert, 1, false);

 for(int i = 0; i < iso_points.vert.size(); ++i){
   CVertex& v = iso_points.vert[i];
   double dist = computeEulerDist(v.P(), original.vert[v.neighbors[0]]);
   v.eigen_confidence = dist;
 }

 normalizeConfidence(iso_points.vert, 0);

 for(int i = 0; i < iso_points.vert.size(); ++i){
   iso_points.vert[i].eigen_confidence = 1 - iso_points.vert[i].eigen_confidence;
 }

 cout<<"save hole confidence" <<endl;
 ofstream out("Data/nbv_output/hole_confidence.txt");
 for (int i = 0; i < iso_points.vert.size(); ++i){
   out<<iso_points.vert[i].eigen_confidence <<endl;
 }
 out.close();
}

/************************************************************************/
/* NBV                                                                  */
/************************************************************************/

void buildGrid()
{
  assert(!iso_points.vert.empty());

  bool use_grid_segment = false; //para->getBool("Run Grid Segment");
  //fix: this should be model->bbox.max
  Point3f bbox_max = iso_points.bbox.max;
  Point3f bbox_min = iso_points.bbox.min;
  //get the whole 3D space that a camera may exist
  //double camera_max_dist = camera_far_dist / predicted_model_size;//global_paraMgr.camera.getDouble("Camera Far Distance") /
    //global_paraMgr.camera.getDouble("Predicted Model Size"); //predicted model size 应该用李昊检测出来的点云大小

  float scan_box_size = camera_far_dist_virtual + 0.5;
  whole_space_box_min = Point3f(-scan_box_size, -scan_box_size, -scan_box_size);
  whole_space_box_max = Point3f(scan_box_size, scan_box_size, scan_box_size);
  whole_space_box.SetNull();
  whole_space_box.Add(whole_space_box_min);
  whole_space_box.Add(whole_space_box_max);

  //compute the size of the 3D space
  Point3f dif = whole_space_box_max - whole_space_box_min;
  //change the whole space box into a cube
  double max_length = std::max(dif.X(), std::max(dif.Y(), dif.Z()));
  whole_space_box_max = whole_space_box_min + Point3f(max_length, max_length, max_length);
  dif = whole_space_box_max - whole_space_box_min;

  //divide the box into grid
  x_max = static_cast<int> (dif.X() / grid_step_size);
  y_max = static_cast<int> (dif.Y() / grid_step_size);
  z_max = static_cast<int> (dif.Z() / grid_step_size);

  int all_max = std::max(std::max(x_max, y_max), z_max);
  x_max = y_max =z_max = all_max+1; // wsh 12-11

  //preallocate the memory
  int max_index = x_max * y_max * z_max;
  view_grid_points.vert.resize(max_index);
  //increase from whole_space_box_min
  for (int i = 0; i < x_max; ++i)
  {
    for (int j = 0; j < y_max; ++j)
    {
      for (int k = 0; k < z_max; ++k)
      {
        //add the grid
        int index = i * y_max * z_max + j * z_max + k;
        //add the center point of the grid
        //CVertex t;
        CVertex &t = view_grid_points.vert[index];
        t.P()[0] = whole_space_box_min.X() + i * grid_step_size;
        t.P()[1] = whole_space_box_min.Y() + j * grid_step_size;
        t.P()[2] = whole_space_box_min.Z() + k * grid_step_size;
        t.m_index = index;
        t.is_view_grid = true;
        //view_grid_points.vert[index] = t;
        view_grid_points.bbox.Add(t.P());
      }
    }
  }
  view_grid_points.vn = max_index;
  cout << "all grid points: " << max_index << endl;
  cout << "resolution: " << x_max << endl;

  bool test_field_segment = false;//para->getBool("Test Other Inside Segment");
  if (field_points.vert.empty())
  {
    test_field_segment = false;
    cout << "field points empty" << endl;
  }
  //distinguish the inside or outside grid

  if (test_field_segment){
    computeAnnNeigbhors(field_points.vert, view_grid_points.vert, 1, false);
  }else{
    computeAnnNeigbhors(iso_points.vert, view_grid_points.vert, 1, false);
  }

  double grid_step_size2 = grid_step_size * grid_step_size;
  for (int i = 0; i < view_grid_points.vert.size(); ++i)
  {
    Point3f &t = view_grid_points.vert[i].P();
    if (!view_grid_points.vert[i].neighbors.empty())
    {
      if (test_field_segment)
      {
        CVertex &nearest = field_points.vert[view_grid_points.vert[i].neighbors[0]];
        if (nearest.eigen_confidence > 0)
        {
          view_grid_points.vert[i].is_ray_stop = true; 
        }
      }
      else
      {
        CVertex &nearest = iso_points.vert[view_grid_points.vert[i].neighbors[0]];
        Point3f &v = nearest.P();
        double dist2 = computeEulerDistSquare(t, v);
        Point3f n = nearest.N();
        Point3f l = view_grid_points.vert[i].P() - nearest.P();
        if (n * l < 0.0f && dist2 < grid_step_size2 * 4)
        {
          view_grid_points.vert[i].is_ray_stop = true; 
        }  
      }
    }
  }

  for (int i = 0; i < view_grid_points.vert.size(); ++i)
  {
    Point3f &t = view_grid_points.vert[i].P();
    if (!view_grid_points.vert[i].neighbors.empty())
    {
      if (test_field_segment)
      {
        CVertex &nearest = field_points.vert[view_grid_points.vert[i].neighbors[0]];
        if (nearest.eigen_confidence > 0)
        {
          view_grid_points.vert[i].is_ray_stop = true; 
        }
      }
      else
      {
        CVertex &nearest = iso_points.vert[view_grid_points.vert[i].neighbors[0]];
        Point3f &v = nearest.P();
        double dist = computeEulerDist(t, v);
        Point3f n = nearest.N();
        Point3f l = view_grid_points.vert[i].P() - nearest.P();
        if ((n * l < 0.0f && dist < grid_step_size * 2)
          /*|| (test_other_segment && dist < grid_step_size / 2)*/) //wsh change
        {
          view_grid_points.vert[i].is_ray_stop = true; 
        }  
      }
    }
  }
  if (use_grid_segment)
  {
    for (int i = 0; i < view_grid_points.vert.size(); ++i)
    {
      CVertex &t = view_grid_points.vert[i];

      if (!t.is_ray_stop)
      {
        t.is_ignore = true;
      }
    }
  }

  saveMesh("Data/nbv_output/build_grid.ply", view_grid_points);
  cout<<"save build_grid.ply done!" <<endl;
}

void propagate()
{
  bool use_propagate_one_point = false;//para->getBool("Run Propagate One Point");
  bool use_max_propagation = true; //para->getBool("Use Max Propagation");

  double predicted_model_length = predicted_model_size;
  double n_dist = camera_far_dist; //.camera.getDouble("Camera Near Distance");
  double f_dist = camera_near_dist; //.camera.getDouble("Camera Far Distance");
  //normalize near and far dist to virtual environment
  n_dist /= predicted_model_length;
  f_dist /= predicted_model_length;

  if (!view_grid_points.vert.empty())
  {
    for (int i = 0; i < view_grid_points.vert.size(); i++)
    {
      CVertex& t = view_grid_points.vert[i];
      t.eigen_confidence = 0.0;
      t.N() = Point3f(0., 0., 0.);
      t.weight_sum = 0.0;
    }
  }

  if (!nbv_candidates.vert.empty()) 
    nbv_candidates.vert.clear();


  double camera_max_dist = camera_far_dist_virtual; //global_paraMgr.camera.getDouble("Camera Far Distance") /
    //global_paraMgr.camera.getDouble("Predicted Model Size");

  int max_steps = static_cast<int>(camera_max_dist / grid_step_size);
  max_steps *= max_ray_steps; //para->getDouble("Max Ray Steps Para"); //wsh

  double ray_density_para = max_ray_steps; //para->getDouble("Max Ray Steps Para");

  int target_index = 0;
  if (use_propagate_one_point)
  {
    target_index = 0; //para->getDouble("Propagate One Point Index");

    if (target_index < 0 || target_index >= iso_points.vert.size())
    {
      srand(time(NULL)); 
      target_index = rand() % iso_points.vert.size();
    }

    cout << "propagate one point index: " << target_index << endl;
  }

  //parallel
  double gaussian_para = 4;//control steepness
  int iso_points_size = iso_points.vert.size();
  double optimal_D = (n_dist + f_dist) / 2.0f;
  double half_D = n_dist;
  double half_D2 = half_D * half_D;
  double gaussian_term = - gaussian_para / half_D2; 
  double sigma = 30.f; //global_paraMgr.norSmooth.getDouble("Sharpe Feature Bandwidth Sigma");
  double sigma_threshold = pow(max(1e-8, 1-cos(sigma / 180.0 * 3.1415926)), 2);

  double ray_resolution_para = 0.511111111111111; //para->getDouble("Ray Resolution Para");
  double angle_delta = (grid_step_size * ray_resolution_para) / camera_max_dist;
  cout << "Angle Delta/resolution:  " << angle_delta << " , " << PI / angle_delta << endl;

#ifdef LINKED_WITH_TBB
  //tbb::mutex _mutex;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, iso_points_size), 
    [&](const tbb::blocked_range<size_t>& r)
  {
    for (size_t i = r.begin(); i < r.end(); ++i)
    {
      vector<int> hit_grid_indexes;
      CVertex &v = iso_points.vert[i];

      if (use_propagate_one_point && v.m_index != target_index)
        continue;

      v.is_ray_hit = true;
      //get the x,y,z index of each iso_points
      int t_indexX = static_cast<int>( ceil((v.P()[0] - whole_space_box_min.X()) / grid_step_size ));
      int t_indexY = static_cast<int>( ceil((v.P()[1] - whole_space_box_min.Y()) / grid_step_size ));
      int t_indexZ = static_cast<int>( ceil((v.P()[2] - whole_space_box_min.Z()) / grid_step_size ));
      //next point index along the ray, pay attention , index should be stored in double ,used in integer
      double n_indexX, n_indexY, n_indexZ;
      //compute the delta of a,b so as to traverse the whole sphere
      //loop for a, b
      double a = 0.0f, b = 0.0f;
      double l = 0.0f;
      double x = 0.0f, y = 0.f, z = 0.0f;
      //for DDA algorithm
      double length = 0.0f;
      double deltaX, deltaY, deltaZ;
      //1. for each point, propagate to all discrete directions
      for (a = 0.0f; a < PI; a += angle_delta)
      {
        l = sin(a); y = cos(a);
        for (b = 0.0f; b < 2 * PI; b += angle_delta)
        {
          //now the propagate direction is Point3f(x, y, z)
          x = l * cos(b); z = l * sin(b);
          //reset the next grid indexes
          n_indexX = t_indexX; n_indexY = t_indexY; n_indexZ = t_indexZ;
          //2. compute the next grid indexes
          length = getAbsMax(x, y, z);
          deltaX = x / length; 
          deltaY = y / length;
          deltaZ = z / length;

          for (int k = 0; k <= max_steps; ++k)     
          {
            n_indexX = n_indexX + deltaX;
            n_indexY = n_indexY + deltaY;
            n_indexZ = n_indexZ + deltaZ;
            int index = round(n_indexX) * y_max * z_max + round(n_indexY) * z_max + round(n_indexZ);

            if (index >= view_grid_points.vert.size())  break;
            //if the direction is into the model, or has been hit, then stop tracing
            if (view_grid_points.vert[index].is_ray_stop) break;            
            if (view_grid_points.vert[index].is_ray_hit)  continue;

            //_mutex.lock();
            //if the grid get first hit 
            view_grid_points.vert[index].is_ray_hit = true;
            //1. set the confidence of the grid center
            CVertex& t = view_grid_points.vert[index];
            Point3f diff = t.P() - v.P();
            double dist2 = diff.SquaredNorm();
            double dist = sqrt(dist2);

            Point3f view_direction = diff.Normalize();
            double opt_dist = dist - optimal_D;
            double coefficient1 = exp(opt_dist * opt_dist * gaussian_term);
            double coefficient2 = exp(-pow(1-v.N() * view_direction, 2) / sigma_threshold);

            float iso_confidence = 1 - v.eigen_confidence;     
            float confidence_weight = coefficient1 * coefficient2;

            if (use_max_propagation)
            {
              //t.eigen_confidence = (std::max)(float(t.eigen_confidence), float(confidence_weight * iso_confidence));          
              if (confidence_weight * iso_confidence > t.eigen_confidence)
              {
                t.eigen_confidence = confidence_weight * iso_confidence;
                t.N() = (v.P()-t.P()).Normalize();
                t.remember_iso_index = v.m_index;
              }
            }
            else
            {
              t.eigen_confidence += coefficient1 * iso_confidence;      
            }

            // record hit_grid center index
            hit_grid_indexes.push_back(index);                        
          }//end for k
        }// end for b
      }//end for a

      if (hit_grid_indexes.size() > 0)
      {
        setGridUnHit(hit_grid_indexes);
        hit_grid_indexes.clear();
      }

      if (use_propagate_one_point)  break;
    }//end for iso_points
  });
#else
  for (int i = 0 ;i < iso_points.vert.size(); ++i)//fix: < iso_points->vert.size()    
  {
    //    cout << "index" << i << endl;
    vector<int> hit_grid_indexes;

    CVertex &v = iso_points.vert[i];
    //t is the ray_start_point
    v.is_ray_hit = true;
    //ray_hit_nbv_grids->vert.push_back(v);

    //get the x,y,z index of each iso_points
    int t_indexX = static_cast<int>( ceil((v.P()[0] - whole_space_box_min.X()) / grid_step_size ));
    int t_indexY = static_cast<int>( ceil((v.P()[1] - whole_space_box_min.Y()) / grid_step_size ));
    int t_indexZ = static_cast<int>( ceil((v.P()[2] - whole_space_box_min.Z()) / grid_step_size ));
    //next point index along the ray, pay attention , index should be stored in double ,used in integer
    double n_indexX, n_indexY, n_indexZ;

    //loop for a, b
    double a = 0.0f, b = 0.0f;
    double l = 0.0f;
    double x = 0.0f, y = 0.f, z = 0.0f;
    //for DDA algorithm
    //int stepX = 0, stepY = 0, stepZ = 0;

    double length = 0.0f;
    double deltaX, deltaY, deltaZ;

    //double half_D = optimal_D / 2.0f;
    double optimal_D = (n_dist + f_dist) / 2.0f;
    double half_D = optimal_D / 2.0f; //wsh    
    double half_D2 = half_D * half_D;
    //for debug

    double sigma = 30.f; //global_paraMgr.norSmooth.getDouble("Sharpe Feature Bandwidth Sigma");
    double sigma_threshold = pow(max(1e-8, 1-cos(sigma/180.0*3.1415926)), 2);

    //1. for each point, propagate to all discrete directions
    for (a = 0.0f; a < PI; a += angle_delta)
    {
      l = sin(a); y = cos(a);
      for (b = 0.0f; b < 2 * PI; b += angle_delta)
      {
        //now the propagate direction is Point3f(x, y, z)
        x = l * cos(b); z = l * sin(b);
        //reset the next grid indexes
        n_indexX = t_indexX; n_indexY = t_indexY; n_indexZ = t_indexZ;
        //2. compute the next grid indexes
        length = getAbsMax(x, y, z);
        deltaX = x / length; 
        deltaY = y / length;
        deltaZ = z / length;

        //int hit_stop_time = 0;
        for (int k = 0; k <= max_steps; ++k)
          //for (int k = 0; k <= 100000; ++k)
          //while (1)        
        {
          n_indexX = n_indexX + deltaX;
          n_indexY = n_indexY + deltaY;
          n_indexZ = n_indexZ + deltaZ;
          int index = round(n_indexX) * y_max * z_max + round(n_indexY) * z_max + round(n_indexZ);

          if (index >= view_grid_points.vert.size())
          {
            break;
          }
          //if the direction is into the model, or has been hit, then stop tracing
          if (view_grid_points.vert[index].is_ray_stop)
          {
            break;
          }

          if (view_grid_points.vert[index].is_ray_hit)  continue;

          //if the grid get first hit 
          view_grid_points.vert[index].is_ray_hit = true;
          //1. set the confidence of the grid center
          CVertex& t = view_grid_points.vert[index];
          //double dist = GlobalFun::computeEulerDist(v.P(), t.P());
          Point3f diff = t.P() - v.P();
          double dist2 = diff.SquaredNorm();
          double dist = sqrt(dist2);

          Point3f view_direction = diff.Normalize();
          double opt_dist = dist - optimal_D;
          double coefficient1 = exp(opt_dist * opt_dist * gaussian_term);
          double coefficient2 = exp(-pow(1-v.N()*view_direction, 2)/sigma_threshold);

          float iso_confidence = 1 - v.eigen_confidence;
          float view_weight = iso_confidence * coefficient2;   
          float confidence_weight = coefficient1 * coefficient2;

          if (use_max_propagation)
          {
            //t.eigen_confidence = (std::max)(float(t.eigen_confidence), float(confidence_weight * iso_confidence));          
            if (confidence_weight * iso_confidence > t.eigen_confidence)
            {
              t.eigen_confidence = confidence_weight * iso_confidence;
              t.N() = (v.P()-t.P()).Normalize();
              t.remember_iso_index = v.m_index;
            }
          }
          else
          {
            //t.eigen_confidence += coefficient1 * iso_confidence;
            t.eigen_confidence += coefficient1 * 1.0;
          }
          // record hit_grid center index
          hit_grid_indexes.push_back(index);
        }//end for k
      }// end for b
    }//end for a

    if (hit_grid_indexes.size() > 0)
    {
      setGridUnHit(hit_grid_indexes);
      hit_grid_indexes.clear();
    }

    if (use_propagate_one_point)
    {
      break;
    }
  }//end for iso_points
#endif

  normalizeConfidence(view_grid_points.vert, 0.);

  std::cout<<"save view_grid_points with confidence after propagate" <<endl;
  ofstream out("Data/nbv_output/view_grid_points_with_confidence.txt");
  for (int i = 0; i < view_grid_points.vert.size(); ++i){
    out<<view_grid_points.vert[i].eigen_confidence <<endl;
  }
  out.close();
}

void viewExtractionIntoBins(int view_bin_each_axis)
{
  nbv_candidates.vert.clear();

  Point3f diff = whole_space_box_max - whole_space_box_min;
  double bin_length_x = diff.X() / view_bin_each_axis;
  double bin_length_y = diff.Y() / view_bin_each_axis;
  double bin_length_z = diff.Z() / view_bin_each_axis;

  //dynamic allocate memory
  int bin_confidence_size = view_bin_each_axis * view_bin_each_axis * view_bin_each_axis;
  float *bin_confidence = new float[bin_confidence_size];
  memset(bin_confidence, 0, bin_confidence_size * sizeof(float));

  int ***view_bins;
  view_bins = new int **[view_bin_each_axis];
  for (int i = 0; i < view_bin_each_axis; ++i)
  {
    view_bins[i] = new int *[view_bin_each_axis];
    for (int j = 0; j < view_bin_each_axis; ++j)
    {
      view_bins[i][j] = new int[view_bin_each_axis]();
    }
  }

  //process each iso_point
  int index = 0; 
  for (int i = 0; i < view_grid_points.vert.size(); ++i)
  {
    CVertex &v = view_grid_points.vert[i];

    int t_indexX = static_cast<int>( floor((v.P()[0] - whole_space_box_min.X()) / bin_length_x ));
    int t_indexY = static_cast<int>( floor((v.P()[1] - whole_space_box_min.Y()) / bin_length_y ));
    int t_indexZ = static_cast<int>( floor((v.P()[2] - whole_space_box_min.Z()) / bin_length_z ));

    t_indexX = (t_indexX >= view_bin_each_axis ? (view_bin_each_axis-1) : t_indexX);
    t_indexY = (t_indexY >= view_bin_each_axis ? (view_bin_each_axis-1) : t_indexY);
    t_indexZ = (t_indexZ >= view_bin_each_axis ? (view_bin_each_axis-1) : t_indexZ);

    int idx = t_indexY * view_bin_each_axis * view_bin_each_axis
      + t_indexZ * view_bin_each_axis + t_indexX;

    if (v.eigen_confidence > bin_confidence[idx])
    {
      bin_confidence[idx] = v.eigen_confidence;
      view_bins[t_indexX][t_indexY][t_indexZ] = v.m_index;
    }
  }

  //put bin view into nbv_candidates
  //fixme: view_bin one value uninitialized
  for (int i = 0; i < view_bin_each_axis; ++i)
  {
    for (int j = 0; j < view_bin_each_axis; ++j)
    {
      for (int k = 0; k < view_bin_each_axis; ++k)
      {
        if (view_bins[i][j][k] < 0  || view_bins[i][j][k] > view_grid_points.vert.size() - 1)
          continue;
        else
          nbv_candidates.vert.push_back(view_grid_points.vert[view_bins[i][j][k]]);
      }
    }
  }
  nbv_candidates.vn = nbv_candidates.vert.size();

  //delete unqualified candidates
  double confidence_threshold = 0.6f;//para->getDouble("Confidence Filter Threshold");
  double camera_far_dist = camera_far_dist_virtual;//global_paraMgr.camera.getDouble("Camera Far Distance")
    // /global_paraMgr.camera.getDouble("Predicted Model Size");
  double camera_near_dist = camera_near_dist_virtual;//global_paraMgr.camera.getDouble("Camera Near Distance") 
    // /global_paraMgr.camera.getDouble("Predicted Model Size");

  int nbv_candidate_num = 0;
  for (int i = 0; i < nbv_candidates.vert.size(); i++)
  {
    CVertex& v = nbv_candidates.vert[i];
    double dist_to_correspondese = computeEulerDistSquare(v.P(), iso_points.vert[v.remember_iso_index].P());

    if ( /*dist_to_correspondese <= camera_near_dist
         || dist_to_correspondese >= camera_far_distviewpru
         || */isPointInBoundingBox(v.P(), &original, bin_length_x))//TODO:用当前点云的bounding box
    {
      v.is_ignore = true;
    }
    else
    {
      nbv_candidate_num++;
    }
  }

  deleteIgnore(&nbv_candidates);

  delete [] bin_confidence;

  for (int i = 0; i < view_bin_each_axis; ++i)
  {
    for (int j = 0; j < view_bin_each_axis; ++j)
    {
      delete[] view_bins[i][j];
    }
    delete view_bins[i];
  }
  delete view_bins;
}

bool cmp(const CVertex &v1, const CVertex &v2)
{
  //in ascending order
  return v1.eigen_confidence > v2.eigen_confidence;
}

void viewPrune()
{
  Point3f diff = whole_space_box_max - whole_space_box_min;
  double view_prune_radius = diff.X() / 6;
  double prune_confidence_threshold = 0.9; //global_paraMgr.nbv.getDouble("View Prune Confidence Threshold");

  computeBallNeighbors(&nbv_candidates, NULL, view_prune_radius, nbv_candidates.bbox);
  sort(nbv_candidates.vert.begin(), nbv_candidates.vert.end(), cmp);

  double radius = 0.08;//10.0 * global_paraMgr.data.getDouble("CGrid Radius");
  double radius2 = radius * radius;

  for (int i = 0; i < nbv_candidates.vert.size(); ++i)
  {
    CVertex &v = nbv_candidates.vert[i];
    //cout << "!!!!!!!!!!!!!!!!!!candidate confidence: " << v.eigen_confidence << endl;
    //if the point has been ignored, then skip it
    if (v.is_ignore)
      continue;

    CVertex& v_iso = iso_points.vert.at(v.remember_iso_index);
    if (v.eigen_confidence < prune_confidence_threshold)
    {
      v.is_ignore = true;
      continue;
    }

    for (int j = 0; j < v.neighbors.size(); ++j)
    {
      CVertex &t = nbv_candidates.vert[v.neighbors[j]];
      if (t.m_index == v.m_index)
        continue;
      else
        t.is_ignore = true;
    }
  }
  deleteIgnore(&nbv_candidates);
  cout << "after View Prune candidates num: " <<nbv_candidates.vert.size() <<endl;

  int topn = 10; //global_paraMgr.nbv.getInt("NBV Top N");
  sort(nbv_candidates.vert.begin(), nbv_candidates.vert.end(), cmp);
  if (nbv_candidates.vert.size() > topn)
  {
    for (int i = 0; i < nbv_candidates.vn; i++)
    {
      CVertex& v = nbv_candidates.vert[i];
      if (i >= topn)
        v.is_ignore = true;
    }
  }
  deleteIgnore(&nbv_candidates);

  //vector<CVertex> new_candidates;
  //for (int i = 0; i < nbv_candidates->vert.size(); ++i)
  //{
  //  CVertex &v = nbv_candidates->vert[i];
  //  //if the point has been ignored, then skip it
  //  if (v.is_ignore)
  //    continue;

  //  Point3f vp = v.P();
  //  int remember_index = v.remember_iso_index;
  //  if (remember_index < 0 || v.remember_iso_index > iso_points->vert.size())
  //  {
  //    continue;
  //  }
  //  Point3f tp = iso_points->vert.at(remember_index).P();
  //  Point3f plane_point(tp.X(), vp.Y(), vp.Z());

  //  if (vp.X() > tp.X())
  //  {
  //    Point3f vector0 = (vp - tp).Normalize();
  //    Point3f vector1 = (plane_point - tp).Normalize();
  //    double angle = GlobalFun::computeRealAngleOfTwoVertor(vector0, vector1);
  //    if (angle > 13)
  //    {
  //      continue;
  //    }
  //  }

  //  //float x_movement;
  //  float x_movement = tp.X() - vp.X();
  //  float new_x = tp.X() + x_movement;
  //  Point3f new_p = Point3f(new_x, plane_point.Y(), plane_point.Z());

  //  CVertex new_v = v;
  //  new_v.P() = new_p;
  //  new_v.N() = (tp - new_p).Normalize();

  //  v.P() = new_v.P();
  //  v.N() = new_v.N();
  //  //new_candidates.push_back(new_v);
  //}
  //GlobalFun::deleteIgnore(nbv_candidates);
  cout << "after top N candidate num: " <<nbv_candidates.vert.size() <<endl;

  //any two nbv should not scan the same two points
  scan_candidates.clear();
  for (int i = 0; i < nbv_candidates.vert.size(); ++i)
    scan_candidates.push_back(make_pair(nbv_candidates.vert[i].P(), nbv_candidates.vert[i].N()));
}

void BaseSampler::AddVert(const CMesh::VertexType &p) 
{
  tri::Allocator<CMesh>::AddVertices(*m,1);
  m->vert.back().ImportData(p);
}

void BaseSampler::AddFace(const CMesh::FaceType &f, CMesh::CoordType p) 
{
  //cout << "######  3.2.2.1 #########" << endl;

  tri::Allocator<CMesh>::AddVertices(*m,1);

  //cout << "######  3.2.2.2 #########" << endl;
  m->vert.back().P() = f.P(0)*p[0] + f.P(1)*p[1] +f.P(2)*p[2];
  m->vert.back().N() = f.V(0)->N()*p[0] + f.V(1)->N()*p[1] + f.V(2)->N()*p[2];

  /*   if (qualitySampling)	
  m->vert.back().Q() = f.V(0)->Q()*p[0] + f.V(1)->Q()*p[1] + f.V(2)->Q()*p[2];*/
}


///************************************************************************/
///* Main Function                                                        */
///************************************************************************/
//int main(int argc, const char *argv)
//{
//  /*load mesh*/
//  loadMesh("data/test_original.ply", original);
//  cout<<original.vert.size() <<endl;
//  normalizeROSA_Mesh(original);
//
//  /*iso points*/
//  saveMesh("poisson_in.ply", original);
//  runPoissonExe();
//  extractPoissonConfidence();
//  extractIsoPoints();
//  computeHoleConfidence();
//
//  /*nbv*/
//  buildGrid();
//  propagate();
//  viewExtractionIntoBins(view_bin_each_axis);
//  viewPrune();
//
//  ofstream out("NBV.txt");
//  for (int i = 0; i < scan_candidates.size(); ++i){
//    cout<<scan_candidates[i].first.X() <<" " <<scan_candidates[i].first.Y() <<" " <<scan_candidates[i].first.Z() <<endl
//      <<scan_candidates[i].second.X() <<" " <<scan_candidates[i].second.Y() <<" " <<scan_candidates[i].second.Z() <<endl;
//  }
//  out.close();
//
//  clearCMesh(nbv_candidates);
//  for (int i = 0; i < scan_candidates.size(); ++i){
//    CVertex t;
//    t.m_index = i;
//    t.P() = scan_candidates[i].first;
//    t.N() = scan_candidates[i].second;
//    nbv_candidates.vert.push_back(t);
//    nbv_candidates.bbox.Add(t.P());
//  }
//  nbv_candidates.vn = nbv_candidates.vert.size();
//  saveMesh("NBV.ply", nbv_candidates);
//
//  system("PAUSE");
//  return 0;
//}