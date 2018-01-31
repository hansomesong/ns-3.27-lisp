/*
 * mapping-socket-msg-header.h
 *
 *  Created on: Apr 3, 2017
 *      Author: qsong
 */

#ifndef SRC_INTERNET_MODEL_LISP_DATA_PLANE_MAPPING_SOCKET_MSG_HEADER_H_
#define SRC_INTERNET_MODEL_LISP_DATA_PLANE_MAPPING_SOCKET_MSG_HEADER_H_

#include "ns3/header.h"
#include "ns3/object.h"

namespace ns3 {

class MappingSocketMsgHeader : public Header
{
public:
    /**
     * \brief Get the type identifier.
     * \return type identifier
     */
    static TypeId GetTypeId (void);

    MappingSocketMsgHeader ();
    virtual
    ~MappingSocketMsgHeader ();

    /**
     * \brief Print some informations about the packet.
     * \param os output stream
     * \return info about this packet
     */
    virtual void Print (std::ostream& os) const;

    /**
     * \brief Get the serialized size of the packet.
     * \return size
     */
    virtual uint32_t GetSerializedSize (void) const;

    /**
     * \brief Serialize the packet.
     * \param start Buffer iterator
     */
    virtual void Serialize (Buffer::Iterator start) const;

    /**
     * \brief Deserialize the packet.
     * \param start Buffer iterator
     * \return size of the packet
     */
    virtual uint32_t Deserialize (Buffer::Iterator start);

    /**
     * \brief Return the instance type identifier.
     * \return instance type ID
     */
    virtual TypeId GetInstanceTypeId (void) const;

    void SetMapVersion (uint8_t mapVersion);
    uint8_t GetMapVersion (void);

    void SetMapType (uint16_t mapType);
    uint16_t GetMapType (void);

    void SetMapFlags (uint32_t mapFlags);
    uint32_t GetMapFlags (void);

    void SetMapAddresses (uint16_t mapAddresses);
    uint16_t GetMapAddresses (void);

    void SetMapVersioning (uint16_t mapVersioning);
    uint16_t GetMapVersioning (void);

    void SetMapRlocCount (uint32_t mapRlocCount);
    uint32_t GetMapRlocCount (void);
  private:
    uint8_t m_mapVersion; /* ? future binary compatibility */
    uint16_t m_mapType; /* message type */
    uint32_t m_mapFlags; /* flags */
    uint16_t m_mapAddresses;
    uint16_t m_mapVersioning; /* Map Version number */
    uint32_t m_mapRlocCount; /* Number of RLOCs appended*/

};
} /* namespace ns3 */

#endif /* SRC_INTERNET_MODEL_LISP_DATA_PLANE_MAPPING_SOCKET_MSG_HEADER_H_ */
