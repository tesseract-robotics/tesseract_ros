# Possible Override Types
# Do not apply contact margin data
uint8 NONE=0
# Replace the contact manager's CollisionMarginData
uint8 REPLACE=1
# Modify the contact managers default margin and pair margin.
# This will preserve existing pairs not being modified by the provided margin data.
# If a pair already exist it will be updated with the provided margin data.
uint8 MODIFY=2

# one of the option above
uint8 type
