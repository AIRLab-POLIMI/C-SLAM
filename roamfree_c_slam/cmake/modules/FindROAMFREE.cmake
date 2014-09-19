# - ROAMFREE library and header paths

# Note:
#
# At the moment ROAMFREE does not have a proper project-config file. 
# The user has to fill the ROAMFREE_BUILD_PATH and ROAMFREE_SRC_PATH
# Please be patient until we fix this.

# ---- fill the next two lines ------------------------------------------------

SET(ROAMFREE_SRC_PATH  /home/dave/ROAMFREE/src/)
SET(ROAMFREE_BUILD_PATH /home/dave/ROAMFREE/Debug/)

# ---- do not edit after here -------------------------------------------------

SET(LIB_ROAMESTIMATION LIB_ROAMESTIMATION-NOTFOUND)
SET(LIB_ROAMIMU LIB_ROAMIMU-NOTFOUND)

FIND_LIBRARY(LIB_ROAMESTIMATION NAMES ROAMestimation ROAMestimation_d PATHS ${ROAMFREE_BUILD_PATH}/lib)
FIND_LIBRARY(LIB_ROAMIMU NAMES ROAMimu ROAMimu_d PATHS ${ROAMFREE_BUILD_PATH}/lib)

SET(ROAMFREE_LIBRARIES ${LIB_ROAMESTIMATION} ${LIB_ROAMIMU})

SET(ROAMFREE_INCLUDE_DIR ${ROAMFREE_SRC_PATH}/ROAMFREE ${ROAMFREE_SRC_PATH}/g2o ${ROAMFREE_BUILD_PATH}/g2o)

SET(ROAMFREE_FOUND TRUE) 