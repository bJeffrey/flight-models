#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/websocket/stream.hpp>
#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;
using Json = nlohmann::json;

namespace {

using Ws = websocket::stream<tcp::socket>;

std::string read_text(Ws& ws) {
  beast::flat_buffer buffer;
  ws.read(buffer);
  return beast::buffers_to_string(buffer.data());
}

std::string expect_reply(Ws& ws, const std::string& prefix, const std::string& action) {
  const bool expecting_error = prefix == "-ERR";
  while (true) {
    const std::string reply = read_text(ws);
    if (reply.rfind(prefix, 0) == 0) {
      return reply;
    }
    if (reply.rfind("-ERR", 0) == 0 || expecting_error) {
      throw std::runtime_error(action + " expected " + prefix + ", received: " + reply);
    }
  }
}

void publish(Ws& ws, const Json& payload, const std::string& expected,
             const std::string& description) {
  ws.write(asio::buffer("PUB contract.prd " + payload.dump()));
  const auto reply = expect_reply(ws, expected, description);
  if (expected == "-ERR" && reply.find("Timestamp") == std::string::npos)
    throw std::runtime_error("expected missing Timestamp rejection, received: " + reply);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: uci_fixture_probe <host> <port> <prd-fixture.json>\n";
    return 2;
  }

  try {
    std::ifstream fixture_file(argv[3]);
    if (!fixture_file) {
      throw std::runtime_error("cannot open PRD fixture: " + std::string(argv[3]));
    }
    Json fixture;
    fixture_file >> fixture;

    asio::io_context context;
    tcp::resolver resolver(context);
    Ws ws(context);
    asio::connect(ws.next_layer(), resolver.resolve(argv[1], argv[2]));
    ws.set_option(websocket::stream_base::decorator([](websocket::request_type& request) {
      request.set(boost::beast::http::field::sec_websocket_protocol, "owp");
    }));
    ws.handshake(argv[1], "/owp");

    const std::string init =
        R"(INIT {"versions":["1.0"],"schema":"002.5.0","verbose":true,"service_id":"contract-test"})";
    ws.write(asio::buffer(init));
    expect_reply(ws, "+OK", "INIT acknowledgment");
    expect_reply(ws, "INFO ", "INIT information");

    publish(ws, fixture, "+OK", "nominal PRD publication");

    // Both choices are schema-valid; consumer identity interpretation is a separate contract.
    Json alternate_source = fixture;
    auto& source = alternate_source["PositionReportDetailed"]["MessageData"]
                                      ["PositionReportData"][0]["PositionSource"];
    const bool supplied_subsystem = source.contains("SubsystemID");
    const std::string supplied_choice = supplied_subsystem ? "SubsystemID" : "SystemID";
    const std::string alternate_choice = supplied_subsystem ? "SystemID" : "SubsystemID";
    const Json identity = source.at(supplied_choice);
    source = {{alternate_choice, identity}};
    publish(ws, alternate_source, "+OK", "alternate PositionSource PRD publication");

    Json missing_timestamp = fixture;
    missing_timestamp["PositionReportDetailed"]["MessageHeader"].erase("Timestamp");
    publish(ws, missing_timestamp, "-ERR", "missing MessageHeader.Timestamp rejection");

    // A schema error may close the ASB connection. The negative case is last,
    // so socket destruction completes cleanup without another protocol exchange.
    std::cout << "Sleet accepted both PositionSource PRDs and rejected missing header timestamp\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "uci_fixture_probe: " << error.what() << '\n';
    return 1;
  }
}
