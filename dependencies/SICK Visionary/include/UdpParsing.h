#include <map>
#include <memory>
#include <vector>

#include <ITransport.h>
#include <VisionaryData.h>

namespace visionary {
ITransport::ByteBuffer reassembleFragments(const std::map<std::uint16_t, ITransport::ByteBuffer>& fragmentMap);

bool parseSegmBinaryData(std::vector<std::uint8_t>::iterator itBuf,
                         std::size_t                         bufferSize,
                         std::shared_ptr<VisionaryData>      m_dataHandler);

bool parseUdpBlob(std::vector<std::uint8_t> buffer, std::shared_ptr<VisionaryData> m_dataHandler);

} // namespace visionary
