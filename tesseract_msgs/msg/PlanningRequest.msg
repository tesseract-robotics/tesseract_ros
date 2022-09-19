tesseract_msgs/EnvironmentState environment_state
tesseract_msgs/EnvironmentCommand[] commands # Additional Commands to be applied to environment prior to planning

string name # The name of the taskflow to use
string instructions # This should an xml string of the command language instructions
bool debug # Enable debug content
bool save_io # Enable saving of input and output for task infos

tesseract_msgs/PlannerProfileRemapping move_profile_remapping
tesseract_msgs/PlannerProfileRemapping composite_profile_remapping

string TRAJOPT_PIPELINE_NAME = TrajOptPipeline
string TRAJOPT_IFOPT_PIPELINE_NAME = TrajOptIfoptPipeline
string OMPL_PIPELINE_NAME = OMPLPipeline
string DESCARTES_PIPELINE_NAME = DescartesPipeline
string FREESPACE_PIPELINE_NAME = FreespacePipeline
string CARTESIAN_PIPELINE_NAME = CartesianPipeline

# Raster pipeline using freespace pipeline for transitions
string RASTER_FT_PIPELINE_NAME = RasterFtPipeline

# Raster pipeline using cartesian pipeline for transitions
string RASTER_CT_PIPELINE_NAME = RasterCtPipeline

# Raster pipeline performs global plan first then macro planning using freespace pipeline for transitions
string RASTER_FT_G_PIPELINE_NAME = RasterFtGlobalPipeline

# Raster Pipeline performs global plan first then macro planning using cartesian pipeline for transitions
string RASTER_CT_G_PIPELINE_NAME = RasterCtGlobalPipeline

# Raster only pipeline using freespace pipeline for transitions
string RASTER_FT_O_PIPELINE_NAME = RasterFtOnlyPipeline

# Raster only pipeline using cartesian pipeline for transitions
string RASTER_CT_O_PIPELINE_NAME = RasterCtOnlyPipeline

# Raster only pipeline performs global plan first then macro planning using freespace pipeline for transitions
string RASTER_FT_O_G_PIPELINE_NAME = RasterFtOnlyGlobalPipeline

# Raster only pipeline performs global plan first then macro planning using cartesian pipeline for transitions
string RASTER_CT_O_G_PIPELINE_NAME = RasterCtOnlyGlobalPipeline
