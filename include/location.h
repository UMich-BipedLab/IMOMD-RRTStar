#ifndef LOCATION_STRUCT_H
#define LOCATION_STRUCT_H

typedef struct location
{
    size_t id;
    double latitude;
    double longitude;

    bool operator==(const location& loc) const
    {
        return (id == loc.id);
    }
} location_t;

#endif /* LOCATION_STRUCT_H */