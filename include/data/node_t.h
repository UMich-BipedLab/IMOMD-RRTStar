#ifndef NODE_STRUCT_H
#define NODE_STRUCT_H

#include <cstddef>

typedef struct node
{
    size_t id;
    double x;
    double y;
    double z;

    bool operator==(const node& rhs) const
    {
        return (id == rhs.id);
    }
} node_t;

#endif /* NODE_STRUCT_H */