// #ifndef PLATFORMINTERFACEGADAL_HPP
#define PLATFORMINTERFACEGADAL_HPP

#include <iomanip> // std::setprecision

#define FILE_INTERFACE_READ "Results"

#ifndef FILE_INTERFACE_READ
#include "OOPS, NEED TO DEFINE FILE_INTERFACE_READ!"
#endif

// whether to get node_vnoms from file or hardwire to 1
#define FILE_INTERFACE_VNOM
// the nosbase symbol is used for a model outside GridAPPS-D like the
// 4-bus MATLAB model
// #define SBASE_NONE
// dummy
// #include "helics/application_api/BrokerApp.hpp"
// #include "helics/application_api/CombinationFederate.hpp"
// #include "helics/core/helicsCLI11.hpp"
// #include "helics/core/helics_definitions.hpp"
#include "json.hpp"
#include <cmath>
#include <helics/cpp98/ValueFederate.hpp>
#include <helics/cpp98/helics.hpp>
using json = nlohmann::json;

#include "SharedQueue.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>

class PlatformInterface : public PlatformInterfaceBase {
public:
  PlatformInterface(int argc, char **argv, const double &sbase)
      : PlatformInterfaceBase(argc, argv, sbase) {
    std::string fedinitstring = "--federates=1 --slowresponding";
    double deltat = 1;

    std::filesystem::path inputMapFile =
        std::filesystem::current_path() / "input_mapping.json";
    std::filesystem::path staticInFile =
        std::filesystem::current_path() / "static_inputs.json";
    std::cout << "Reading OEDISI configurations from : " << inputMapFile
              << " and " << staticInFile << std::endl;

    std::ifstream f1(inputMapFile.string());
    inputMap = json::parse(f1);
    std::ifstream f2(staticInFile.string());
    staticInput = json::parse(f2);

    std::cout << sbase << std::endl;
    std::string helicsversion = helicscpp::getHelicsVersionString();

    if (helicsversion.find("error") == std::string::npos) {
      // this has to do with tests passing on CI builds
      std::cout << "Helics version = " << helicsversion << '\n';
    }

    /* Create Federate Info object that describes the federate properties
     * Set federate name and core type from string
     */
    helicscpp::FederateInfo fi("zmq");

    /* Federate init string */
    fi.setCoreInit(fedinitstring);
    fi.setCoreName(staticInput["name"]);

    fi.setProperty(HELICS_PROPERTY_TIME_DELTA, deltat);

    fi.setProperty(HELICS_PROPERTY_INT_MAX_ITERATIONS, 100);

    fi.setProperty(HELICS_PROPERTY_INT_LOG_LEVEL, HELICS_LOG_LEVEL_DEBUG);

    /* Create value federate */
    vfed = new helicscpp::ValueFederate(staticInput["name"], fi);
    std::cout << " Value federate created\n";

    // Set up subscriptions
    sub_topo = vfed->registerSubscription(inputMap["topology"], "");

    sub_V = vfed->registerSubscription(inputMap["voltage_mag"], "V");

    sub_P = vfed->registerSubscription(inputMap["power_real"], "W");

    sub_Q = vfed->registerSubscription(inputMap["power_imag"], "W");

    if (sub_topo.isValid()) {
      std::cout << " Subscription registered for feeder Topology with port "
                << inputMap["topology"] << std::endl;
    }

    if (sub_P.isValid()) {
      std::cout << " Subscription registered for feeder real power P with port "
                << inputMap["power_real"] << std::endl;
    }

    if (sub_Q.isValid()) {
      std::cout
          << " Subscription registered for feeder reactive power Q with port "
          << inputMap["power_imag"] << std::endl;
    }

    if (sub_V.isValid()) {
      std::cout << " Subscription registered for feeder voltage V with port "
                << inputMap["voltage_mag"] << std::endl;
    }

    // Set up publications
    pub_Vmag =
        vfed->registerPublication("voltage_mag", HELICS_DATA_TYPE_STRING);

    pub_Vang =
        vfed->registerPublication("voltage_angle", HELICS_DATA_TYPE_STRING);

    if (pub_Vmag.isValid()) {
      std::cout << " Vmag Publication registered\n";
    }
    if (pub_Vang.isValid()) {
      std::cout << " Vang Publication registered\n";
    }
    try {
      /* Enter initialization state */
      vfed->enterInitializingMode(); // can throw
                                     // helicscpp::InvalidStateTransition
                                     // exception
      std::cout << " Entered initialization state" << std::endl;
    } catch (...) {
      std::cout << " Unknown error in enterInitializingMode" << std::endl;
    }
    try {
      /* Enter execution state */
      vfed->enterExecutingMode(); // can throw helicscpp::InvalidStateTransition
                                  // exception
      std::cout << " Entered execution state\n";
    } catch (...) {
      std::cout << " Unknown error in enterExecutingMode" << std::endl;
    }

    currenttime = 0.0;
    ts = 1;
    std::cout << "Requesting time" << std::endl;
    currenttime = vfed->requestTime(HELICS_TIME_MAXTIME);
    std::cout << "Received time: " << currenttime << std::endl;

    // check whether all subscriptions are updated or not
    // if not request more time
    try {
      while (!sub_topo.isUpdated()) {
        std::cout << "Topology not updated..." << std::endl;
        currenttime = vfed->requestTime(HELICS_TIME_MAXTIME);
      }

      // Get the topology subscription
      std::string str_topo;
      sub_topo.getString(str_topo);
      topo = json::parse(str_topo);

      SDMAP voltages_mag = extractBaseVoltagesMagnitudes(topo);
      SDMAP voltages_ang = extractBaseVoltagesAngles(topo);
      base_voltages = polarMap(voltages_mag, voltages_ang);

      std::cout << "Voltage Nodes : " << base_voltages.size() << std::endl;

      SDMAP powers_real = extractInjections(topo["injections"]["power_real"]);
      SDMAP powers_imag =
          extractInjections(topo["injections"]["power_imaginary"]);
      base_powers = cartMap(powers_real, powers_imag);

      std::cout << "Injection Nodes : " << base_powers.size() << std::endl;

      focus_nodes = filterFocus(topo["injections"]["power_real"]);
      std::cout << "PV+Cap Nodes : " << focus_nodes.size() << std::endl;

      regulators = filterRegulators(topo["incidences"]);
      std::cout << "Regulator Nodes : " << regulators.size() << std::endl;

      for (const auto bus : topo["slack_bus"]) {
        slack_bus.push_back(bus);
      }

      total_power = totalInjections(base_powers);
      Sbase = ceilingPowerOfTen(total_power);
      sbase_ref = &Sbase;
      std::cout << "updated sbase = " << getsbase() << std::endl;

      double average_p = (total_power.real() / Sbase) / base_powers.size();
      double average_q = (total_power.imag() / Sbase) / base_powers.size();
      std::cout << "Average Power : " << average_p << " + i*" << average_q
                << std::endl;

      sigma_v = staticInput["sigma_v"];
      sigma_p = staticInput["sigma_p"];
      sigma_q = staticInput["sigma_q"];

      base_psuedo_sigma_p = sigma_p * std::abs(average_p);
      base_psuedo_sigma_q = sigma_q * std::abs(average_q);
      base_sigma_p = (sigma_p / 10) * std::abs(average_p);
      base_sigma_q = (sigma_q / 10) * std::abs(average_q);

      while (!sub_V.isUpdated()) {
        std::cout << "Voltage not updated..." << std::endl;
        currenttime = vfed->requestTime(HELICS_TIME_MAXTIME);
      }
      // Get the voltage magnitude subscription
      std::string str_v;
      sub_V.getString(str_v);
      json V_meas = json::parse(str_v);
      workQueue.push(V_meas);

      while (!sub_P.isUpdated()) {
        std::cout << "Real Power not updated..." << std::endl;
        currenttime = vfed->requestTime(HELICS_TIME_MAXTIME);
      }
      // Get the real power subscription
      std::string str_p;
      sub_P.getString(str_p);
      json P_meas = json::parse(str_p);
      workQueue.push(P_meas);

      while (!sub_Q.isUpdated()) {
        std::cout << "Imaginary Power not updated..." << std::endl;
        currenttime = vfed->requestTime(HELICS_TIME_MAXTIME);
      }
      // Get the reactive power subscription
      std::string str_q;
      sub_Q.getString(str_q);
      json Q_meas = json::parse(str_q);
      workQueue.push(Q_meas);

      voltages = extractVoltages(V_meas);
      std::cout << "Measured Voltage Nodes : " << voltages.size() << std::endl;

      ratios = getTapRatios(voltages);

      SDMAP measured_p = extractPowers(P_meas);
      std::cout << "Measured Real Power Nodes : " << measured_p.size()
                << std::endl;

      SDMAP measured_q = extractPowers(Q_meas);
      std::cout << "Measured Imag Power Nodes : " << measured_q.size()
                << std::endl;

      powers = cartMap(measured_p, measured_q);

    } catch (...) {

      std::cout << "Error in PlatformInterfaceGADAL initialization"
                << std::endl;
    }
  }

  SDMAP extractPowers(const json &powers) {
    json ids = powers["ids"];
    json values = powers["values"];
    json units = powers["units"];

    double scale = 1.0;
    if (units == "kW" || units == "kVAR")
      scale = 1000.0;

    SDMAP mapping;
    for (size_t i = 0; i < ids.size(); i++) {
      std::string node = ids[i];
      double x = -scale * (double)values[i];
      if (mapping.count(ids[i]) > 0) {
        mapping[ids[i]] += x;
      } else {
        mapping.insert({ids[i], x});
      }
    }
    return mapping;
  };

  SDMAP filterFocus(const json &injections) {
    json ids = injections["ids"];
    json eqids = injections["equipment_ids"];
    json values = injections["values"];
    json units = injections["units"];

    std::vector<double> numbers;
    for (const double val : values) {
      numbers.push_back(std::abs(val));
    }

    std::sort(numbers.begin(), numbers.end(), std::greater<double>());
    for (const auto val : numbers) {
      std::cout << val << ", ";
    }
    double threshold = numbers[5];
    std::cout << "\nFocus Threshold = " << threshold << std::endl;

    double scale = 1.0;
    if (units == "kW" || units == "kVAR")
      scale = 1000.0;

    SDMAP mapping;
    for (size_t i = 0; i < ids.size(); i++) {
      if (std::abs((double)values[i]) >= threshold)
        continue;

      double x = -scale * (double)values[i];
      if (mapping.count(ids[i]) > 0) {
        mapping[ids[i]] += x;
      } else {
        mapping[ids[i]] = x;
      }
    }

    return mapping;
  };

  SSMAP filterRegulators(const json &incidences) {
    json from_eq = incidences["from_equipment"];
    json to_eq = incidences["to_equipment"];
    json ids = incidences["ids"];

    SSMAP mapping;
    for (size_t i = 0; i < ids.size(); i++) {
      std::string node = ids[i];
      std::string eq = ids[i];

      bool found = false;
      SLIST tags = {"reg", "xfm", "tr"};
      for (const auto tag : tags) {
        auto pos = eq.find(tag);
        if (pos != std::string::npos)
          found = true;
      }
      if (!found)
        continue;

      std::string src = splitNode(from_eq[i]);
      std::string dst = splitNode(to_eq[i]);
      if (src.length() < 3 and dst.length() < 3)
        continue;

      bool has_r = src.back() == 'R' or dst.back() == 'R';
      bool has_lv = src.substr(src.length() - 2) == "LV";
      has_lv = has_lv or dst.substr(dst.length() - 2) == "LV";

      if (!has_r and !has_lv)
        continue;

      bool r_at_end = src.back() == 'R';
      bool lv_at_end = src.substr(src.length() - 2) == "LV";
      if (r_at_end or lv_at_end) {
        mapping.insert({dst, src});
        std::cout << src << " -> " << dst << std::endl;
      } else {
        mapping.insert({src, dst});
        std::cout << src << " -> " << dst << std::endl;
      }
    }
    return mapping;
  };

  SDMAP extractInjections(const json &injections) {
    json ids = injections["ids"];
    json eqids = injections["equipment_ids"];
    json values = injections["values"];
    json units = injections["units"];

    double scale = 1.0;
    if (units == "kW" || units == "kVAR")
      scale = 1000.0;

    SDMAP mapping;
    for (size_t i = 0; i < ids.size(); i++) {
      std::string node = ids[i];
      std::string eq = eqids[i];
      double x = -scale * (double)values[i];
      if (mapping.count(ids[i]) > 0) {
        mapping[ids[i]] += x;
      } else {
        mapping.insert({ids[i], x});
      }
    }
    return mapping;
  };

  SDMAP extractVoltages(const json &voltages) {
    json ids = voltages["ids"];
    json values = voltages["values"];

    SDMAP mapping;
    for (size_t i = 0; i < ids.size(); i++) {
      std::string node = ids[i];
      mapping.insert({ids[i], (double)values[i]});
    }
    return mapping;
  };

  SDMAP extractBaseVoltagesMagnitudes(const json &topology) {
    json obj = topology["base_voltage_magnitudes"];
    json ids = obj["ids"];
    json values = obj["values"];

    SDMAP mapping;
    for (size_t i = 0; i < ids.size(); i++) {
      std::string node = ids[i];
      mapping.insert({ids[i], (double)values[i]});
    }
    return mapping;
  };

  SDMAP extractBaseVoltagesAngles(const json &topology) {
    json obj = topology["base_voltage_angles"];
    json ids = obj["ids"];
    json values = obj["values"];

    SDMAP mapping;
    for (size_t i = 0; i < ids.size(); i++) {
      std::string node = ids[i];
      mapping.insert({ids[i], (double)values[i]});
    }
    return mapping;
  };

  SCMAP cartMap(const SDMAP &real, const SDMAP &imag) {
    SCMAP mapping;
    for (const auto &e : real) {
      std::string key = e.first;
      if (imag.count(key) == 0)
        continue;

      std::complex<double> value =
          std::complex<double>(real.at(key), imag.at(key));
      mapping.insert({key, value});
    }
    return mapping;
  }

  SCMAP polarMap(const SDMAP &mag, const SDMAP &ang) {
    SCMAP mapping;
    for (const auto &e : mag) {
      std::string key = e.first;
      if (ang.count(key) == 0)
        continue;
      std::complex<double> value = std::polar(mag.at(key), ang.at(key));
      mapping.insert({key, value});
    }
    return mapping;
  }

  std::complex<double> totalInjections(const SCMAP &injections) {
    std::complex<double> total(0, 0);
    for (const auto power : injections) {
      total += power.second;
    }
    return total;
  }

  double ceilingPowerOfTen(std::complex<double> num) {
    int exponent = std::round(std::log10(std::abs(num)));
    return std::pow(10, exponent + 1);
  }

  std::string splitTag(std::string str) {
    std::string tag = "source_";
    auto pos = str.find(tag);
    if (pos != std::string::npos)
      return str.substr(0, pos + tag.size() + 2); // for V_, T_

    tag = "pseudo_";
    pos = str.find(tag);
    if (pos != std::string::npos)
      return str.substr(0, pos + tag.size() + 2); // for P_, Q_

    tag = "_tap";
    pos = str.find(tag);
    if (pos != std::string::npos)
      return tag;

    pos = str.find("_");
    return str.substr(0, pos + 1);
  }

  std::string splitName(std::string str) {
    std::string tag = "source_";
    auto pos = str.find(tag);
    if (pos != std::string::npos)
      return str.substr(pos + tag.size() + 2); // for V_, T_

    tag = "pseudo_";
    pos = str.find(tag);
    if (pos != std::string::npos)
      return str.substr(pos + tag.size() + 2); // for P_, Q_

    tag = "_tap";
    pos = str.find(tag);
    if (pos != std::string::npos)
      return str.substr(0, pos); // for P_, Q_

    pos = str.find("_");
    return str.substr(pos + 1);
  }

  std::string splitPhase(std::string str) {
    auto pos = str.find(".");
    if (pos != std::string::npos)
      return str.substr(pos);

    return str;
  }

  std::string splitNode(std::string str) {
    auto pos = str.find(".");
    if (pos != std::string::npos)
      return str.substr(0, pos);

    return str;
  }

  SDMAP getTapRatios(SDMAP voltages) {
    ratios;
    for (const auto v : voltages) {
      std::string from = splitNode(v.first);
      std::string phase = splitPhase(v.first);

      if (regulators.count(from) != 0) {
        std::string to = regulators[from] + phase;
        if (voltages.count(to) != 0) {
          double src = v.second / std::abs(base_voltages[v.first]);
          double dst = voltages[to] / std::abs(base_voltages[to]);
          double tap = src / dst;
          ratios[from + phase] = std::round(tap * 100.0) / 100.0;
        }
      }
    }
    return ratios;
  }

  void setupMeasurements() { // measurement ids are loaded here (Done)
    // Adding meas ids
    std::cout << "number of meas id here" << "\n\n";
    std::cout << std::size(meas_zids) << "\n\n";

    for (const auto voltage : voltages) {

      if (base_voltages.count(voltage.first) == 0)
        continue;

      meas_zids.push_back("V_" + voltage.first);

      if (ratios.count(voltage.first) != 0)
        meas_zids.push_back(voltage.first + "_tap");
    }
    for (const auto power : powers) {
      continue; // skip due to singular klu error when they are added

      if (base_powers.count(power.first) == 0)
        continue;

      meas_zids.push_back("P_" + power.first);
      meas_zids.push_back("Q_" + power.first);
    }
    for (const auto voltage : base_voltages) {
      auto it = std::find(slack_bus.begin(), slack_bus.end(), voltage.first);
      if (it != slack_bus.end()) {
        meas_zids.push_back("source_V_" + voltage.first);
        meas_zids.push_back("source_T_" + voltage.first);
      } else {
        meas_zids.push_back("pseudo_P_" + voltage.first);
        meas_zids.push_back("pseudo_Q_" + voltage.first);
      }
    }
    std::cout << "number of meas id after appending pseudo meas" << "\n\n";
    std::cout << std::size(meas_zids) << "\n\n";
    //_________________________________________________________________
  }

  void fillTopo() { // reads nodes list and y matrix info (Done)

    std::cout << std::setw(4) << topo["unique_ids"].size() << "\n\n";
    for (int i = 0; i < topo["base_voltage_magnitudes"]["ids"].size(); i++) {
      string node_name_new = topo["base_voltage_magnitudes"]["ids"][i];
      if (base_voltages.count(node_name_new) == 0)
        continue;
    }
    for (const auto voltage : base_voltages) {
      node_names.push_back(voltage.first);
    }
    std::cout << node_names.size() << "\n\n";
    //___________________________________________________________________
    if (!sparse_impl) {
      std::cout << "Not a sparse matrix implementation" << std::endl;
      std::cout << "admittance_matrix size: "
                << topo["admittance"]["admittance_matrix"].size() << std::endl;
      for (int i = 0; i < topo["admittance"]["admittance_matrix"].size(); i++) {
        for (int j = 0; j < topo["admittance"]["admittance_matrix"][i].size();
             j++) {
          std::cout << "i: " << i << " j: " << j << std::endl;
          double G = (topo["admittance"]["admittance_matrix"][i][j][0]);
          double B = (topo["admittance"]["admittance_matrix"][i][j][1]);
          std::cout << "G: " << G << " B: " << B << std::endl;
          //                    std::cout<< B << std::endl;

          if ((G != 0) || (B != 0)) {
            Yphys[i + 1][j + 1] = complex<double>(G, B);
            if ((i + 1) != (j + 1))
              Yphys[j + 1][i + 1] = complex<double>(G, B);
            // std::cout<< Yphys[i+1][j+1] << std::endl;
          }
        }
      }
    } else {
      std::cout << "Sparse matrix implementation" << std::endl;
      std::cout << "admittance_matrix size: "
                << topo["admittance"]["admittance_list"].size() << std::endl;
      //____________________ now for sparse implementation:
      json obj = topo["admittance"];
      json admittance = obj["admittance_list"];
      json from = obj["from_equipment"];
      json to = obj["to_equipment"];
      for (size_t a = 0; a < admittance.size(); a++) {
        if (base_voltages.count(from[a]) == 0)
          continue;
        if (base_voltages.count(to[a]) == 0)
          continue;

        int i = 0;
        int i_locked = 0;
        int j = 0;
        int j_locked = 0;
        for (auto &node : node_names) {
          if (node == from[a].get<string>()) {
            i_locked = i;
          } else {
            i++;
          }
        }
        for (auto &node : node_names) {
          if (node == to[a].get<string>()) {
            j_locked = j;
          } else {
            j++;
          }
        }

        double G = (admittance[a][0]);
        double B = (admittance[a][1]);
        // need to find i and j?

        if ((G != 0) || (B != 0)) {
          Yphys[i_locked + 1][j_locked + 1] = complex<double>(G, B);
          if ((i_locked + 1) != (j_locked + 1))
            Yphys[j_locked + 1][i_locked + 1] = complex<double>(G, B);
          // std::cout<< Yphys[i+1][j+1] << std::endl;
        }
      }
    }

    // for (int l = 1; l < 5; l++) {
    //     for (int m = 1; m < 5; m++) {
    //         std::cout<< Yphys[l][m] << std::endl;
    //     }
    // }
    // std::cout<< "Last enteries:" << std::endl;
  }

  void fillVnoms() {
    std::cout << "Filling Vnoms" << std::endl;
    node_vnoms = base_voltages; // a complex with both mag and angle
    //__________________________________________________________________
  }

  void fillSensors() {
    // this has to go here due to state_estimator and base class method order
    for (const auto reg : ratios) {
      std::string node = splitNode(reg.first);
      std::string phase = splitPhase(reg.first);
      std::string regid = reg.first + "_tap";
      regid_primnode[regid] = reg.first;
      regid_regnode[regid] = regulators[node] + phase;
      size_t i = node_idxs[reg.first];
      size_t j = node_idxs[regulators[node] + phase];
      Amat[i][j] = 1; // may change
      Amat[j][i] = 1; // stays unity
    }

    for (size_t i = 0; i < meas_zids.size(); i++) {
      std::string zid = meas_zids[i];
      Zary.zids.push_back(zid);
      Zary.zidxs[zid] = i;

      std::string node = splitName(zid);
      Zary.znode1s[zid] = node;
      Zary.znode2s[zid] = node;

      std::string tag = splitTag(zid);
      if (tag == "V_") {
        Zary.ztypes[zid] = "vi";
        Zary.zvals[zid] =
            std::abs(voltages[node]) / std::abs(base_voltages[node]);
        Zary.zsigs[zid] = sigma_v;
        Zary.zpseudos[zid] = false;
        Zary.znomvals[zid] = Zary.zvals[zid];
      }
      if (tag == "_tap") {
        Zary.ztypes[zid] = "aji";
        Zary.zvals[zid] = ratios[node];
        Zary.zsigs[zid] = 0.1 * sigma_v;
        Zary.zpseudos[zid] = false;
        Zary.znomvals[zid] = Zary.zvals[zid];
      }
      if (tag == "V_") {
        Zary.ztypes[zid] = "vi";
        Zary.zvals[zid] =
            std::abs(voltages[node]) / std::abs(base_voltages[node]);
        Zary.zsigs[zid] = sigma_v;
        Zary.zpseudos[zid] = false;
        Zary.znomvals[zid] = Zary.zvals[zid];
      }
      if (tag == "P_") {
        Zary.ztypes[zid] = "Pi";
        Zary.zvals[zid] = powers[node].real() / std::abs(Sbase);
        Zary.zsigs[zid] = 0.01 * Zary.zvals[zid] + base_sigma_p;
        Zary.zpseudos[zid] = false;
        Zary.znomvals[zid] = Zary.zvals[zid];
      }
      if (tag == "Q_") {
        Zary.ztypes[zid] = "Qi";
        Zary.zvals[zid] = powers[node].imag() / std::abs(Sbase);
        Zary.zsigs[zid] = 0.01 * Zary.zvals[zid] + base_sigma_q;
        Zary.zpseudos[zid] = false;
        Zary.znomvals[zid] = Zary.zvals[zid];
      }
      if (tag == "source_V_") {
        Zary.ztypes[zid] = "vi";
        // Zary.zvals[zid] = 1.0;
        Zary.zvals[zid] =
            std::abs(voltages[node]) / std::abs(base_voltages[node]);
        Zary.zsigs[zid] = 5 * sigma_v;
        Zary.zpseudos[zid] = true;
        Zary.znomvals[zid] = Zary.zvals[zid];
      }
      if (tag == "source_T_") {
        Zary.ztypes[zid] = "Ti";
        Zary.zvals[zid] = 0.0;
        Zary.zsigs[zid] = 0.1 * sigma_v;
        Zary.zpseudos[zid] = true;
        Zary.znomvals[zid] = Zary.zvals[zid];
      }
      if (tag == "pseudo_P_") {
        Zary.ztypes[zid] = "Pi";
        Zary.zvals[zid] = powers[node].real() / std::abs(Sbase);
        // Zary.zvals[zid] = (base_powers[node].real() / std::abs(Sbase)) / 2;
        Zary.zsigs[zid] = Zary.zvals[zid] + base_psuedo_sigma_p;
        Zary.zpseudos[zid] = true;
        Zary.znomvals[zid] = Zary.zvals[zid];
      }
      if (tag == "pseudo_Q_") {
        Zary.ztypes[zid] = "Qi";
        Zary.zvals[zid] = powers[node].imag() / std::abs(Sbase);
        // Zary.zvals[zid] = (base_powers[node].imag() / std::abs(Sbase)) / 2;
        Zary.zsigs[zid] = Zary.zvals[zid] + base_psuedo_sigma_q;
        Zary.zpseudos[zid] = true;
        Zary.znomvals[zid] = Zary.zvals[zid];
      }
    }

    std::cout << "Zary size : " << Zary.zids.size() << std::endl;
  }

  bool fillMeasurement() { // Todo --------how to form workqueue
    //_________________________________________________________________
    bool ret = true;

    if (currenttime > Total_ts) {
      // TS: Do we need to stop if current time is greater?
      std::cout << "TIME IS UP " << currenttime << std::endl;
      vfed->finalize();
      delete vfed;
      helicsCloseLibrary();
      return false;
    }

    std::cout << "Current Time: " << currenttime << std::endl;

    json V_message = workQueue.pop();
    json P_message = workQueue.pop();
    json Q_message = workQueue.pop();
    msg_time = V_message["time"];

    std::string voltage_mag;
    sub_V.getString(voltage_mag);
    json V_meas = json::parse(voltage_mag);
    workQueue.push(V_meas);

    std::string power_real;
    sub_P.getString(power_real);
    json P_meas = json::parse(power_real);
    workQueue.push(P_meas);

    std::string power_imag;
    sub_Q.getString(power_imag);
    json Q_meas = json::parse(power_imag);
    workQueue.push(Q_meas);

    meas_timestamp = 0;
    meas_mrids.clear();
    meas_magnitudes.clear();
    meas_timestamp = (uint)(ts++);

    SDMAP measured_v = extractVoltages(V_meas);
    ratios = getTapRatios(measured_v);

    for (const auto v : measured_v) {

      std::string zid = "V_" + v.first;
      meas_mrids.push_back(zid);
      meas_magnitudes[zid] = v.second / std::abs(base_voltages[v.first]);

      if (ratios.count(v.first) != 0) {
        zid = v.first + "_tap";
        std::cout << zid << " updated to " << ratios[v.first] << std::endl;
        meas_mrids.push_back(zid);
        meas_magnitudes[zid] = ratios[v.first];
      }
    }

    SDMAP measured_p = extractPowers(P_meas);
    SDMAP measured_q = extractPowers(Q_meas);
    SCMAP measured_powers = cartMap(measured_p, measured_q);
    for (const auto p : measured_powers) {

      std::string zid = "P_" + p.first;
      meas_mrids.push_back(zid);
      meas_magnitudes[zid] = p.second.real() / Sbase;

      zid = "Q_" + p.first;
      meas_mrids.push_back(zid);
      meas_magnitudes[zid] = p.second.imag() / Sbase;
    }
    return true;
  }

  bool nextMeasurementWaiting() {
    // Always returning false tells the SE work loop to complete an
    // estimate for every measurement. Otherwise it does a single estimate
    // over the entire measurements_data.csv file!
    return false;
  }

  void setupPublishing() {
    std::cout << "Setting up publishing" << std::endl;
    string filename = FILE_INTERFACE_READ;
    filename += "/results_data.csv";
    est_fh.open(filename, std::ofstream::trunc);
    est_fh << "timestamp,";

    for (auto &node_name : node_names)
      est_fh << "vmag_" + node_name + ",";
    uint node_qty = node_names.size();
    uint nctr = 0;
    for (auto &node_name : node_names)
      est_fh << "varg_" + node_name << (++nctr < node_qty ? "," : "\n");

    est_fh.close();
  }

  void publishEstimate(const uint &timestamp, SDMAP &est_v, SDMAP &est_angle,
                       SDMAP &, SDMAP &, SDMAP &est_vmagpu, SDMAP &est_vargpu) {

    string filename = FILE_INTERFACE_READ;
    filename += "/results_data.csv";
    est_fh.open(filename, std::ofstream::app);

    est_fh << timestamp << ',';
    est_fh << std::fixed;
    est_fh << std::setprecision(10);
    std::list<double> est_volt;
    std::list<double> est_ang;
    for (auto &node_name : node_names) {
      est_fh << est_vmagpu[node_name] << ",";
      // std::cout<< "-------------------------------" << std::endl;
      // std::cout<< est_v[node_name] << std::endl;
      // std::cout<< node_name << std::endl;
      est_volt.push_back(est_v[node_name]);
      est_ang.push_back(est_angle[node_name]);
    }
    uint node_qty = node_names.size();
    uint nctr = 0;
    for (auto &node_name : node_names)
      est_fh << est_vargpu[node_name] << (++nctr < node_qty ? "," : "\n");

    est_fh.close();
    time_t now = time(0);

    // convert now to string form
    char *dt = ctime(&now);
    json jmessage_vmag;
    jmessage_vmag["values"] =
        est_volt; //{2331.1810216005406, 2331.177966421088, 2331.1820742949385};
    jmessage_vmag["ids"] = node_names; //{"150.1", "150.2", "150.3"};
    jmessage_vmag["time"] = msg_time;
    jmessage_vmag["units"] = "kV";
    pub_Vmag.publish(jmessage_vmag.dump());

    json jmessage_vang;
    jmessage_vang["values"] =
        est_ang; //{2331.1810216005406, 2331.177966421088, 2331.1820742949385};
    jmessage_vang["ids"] = node_names; //{"150.1", "150.2", "150.3"};
    jmessage_vang["time"] = msg_time;  //"2017-01-01T00:15:00";
    jmessage_vang["units"] = "deg";
    pub_Vang.publish(jmessage_vang.dump());

    currenttime = vfed->requestTime(10000);
  }

  std::vector<string> getZids() { return meas_zids; }

  string getOutputDir() { return FILE_INTERFACE_READ; }

private:
  std::ifstream meas_fh;
  std::ofstream est_fh;
  std::vector<string> meas_zids;
  SIMAP zid_lookup;

  SharedQueue<json> workQueue;
  std::complex<double> total_power;
  SLIST slack_bus;
  double ts;
  double Sbase;
  double sigma_v;
  double sigma_p;
  double sigma_q;
  double base_psuedo_sigma_p;
  double base_psuedo_sigma_q;
  double base_sigma_p;
  double base_sigma_q;
  const bool sparse_impl = true;

  helicscpp::Publication pub_Vmag;
  helicscpp::Publication pub_Vang;

  helicscpp::ValueFederate *vfed;

  HelicsTime currenttime;

  helicscpp::Input sub_topo;
  helicscpp::Input sub_inj;
  helicscpp::Input sub_P;
  helicscpp::Input sub_Q;
  helicscpp::Input sub_V;
  helicscpp::Input sub_A;

  SCMAP base_voltages;
  SCMAP base_powers;
  SDMAP voltages;
  SDMAP focus_nodes;
  SSMAP regulators;
  SDMAP ratios;
  SCMAP powers;

  SLIST node_est_v;

  int Total_ts = 97;

  json topo;
  json inputMap;
  json staticInput;
  json msg_time;
};
