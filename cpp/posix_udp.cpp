
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>

#include <fibre/protocol.hpp>

#define UDP_RX_BUF_LEN	512
#define UDP_TX_BUF_LEN	512


class UDPPacketSender : public PacketSink {
public:
    UDPPacketSender(int socket_fd, struct sockaddr_in6 *si_other) :
        _socket_fd(socket_fd),
        _si_other(si_other)
    {}

    size_t get_mtu() { return UDP_TX_BUF_LEN; }

    int process_packet(const uint8_t* buffer, size_t length) {
        // cannot send partial packets
        if (length > get_mtu())
            return -1;

        int status = sendto(_socket_fd, buffer, length, 0, reinterpret_cast<struct sockaddr*>(_si_other), sizeof(*_si_other));
        return (status == -1) ? -1 : 0;
    }

private:
    int _socket_fd;
    struct sockaddr_in6 *_si_other;
};



int serve_on_udp(unsigned int port) {
    struct sockaddr_in6 si_me, si_other;
    int s;
    socklen_t slen = sizeof(si_other);
    uint8_t buf[UDP_RX_BUF_LEN];

    if ((s=socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP)) == -1)
        return -1;

    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin6_family = AF_INET6;
    si_me.sin6_port = htons(port);
    si_me.sin6_flowinfo = 0;
    si_me.sin6_addr= in6addr_any;
    if (bind(s, reinterpret_cast<struct sockaddr *>(&si_me), sizeof(si_me)) == -1) 
        return -1;

    for (;;) {
        ssize_t n_received = recvfrom(s, buf, sizeof(buf), 0, reinterpret_cast<struct sockaddr *>(&si_other), &slen);
        if (n_received == -1)
            return -1;
        //printf("Received packet from %s:%d\nData: %s\n\n",
        //    inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port), buf);

        UDPPacketSender udp_packet_output(s, &si_other);
        BidirectionalPacketBasedChannel udp_channel(udp_packet_output);
        udp_channel.process_packet(buf, n_received);
    }

    close(s);
}

