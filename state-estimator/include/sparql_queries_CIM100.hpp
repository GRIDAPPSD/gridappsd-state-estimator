#ifndef SPARQL_QUERIES_HPP
#define SPARQL_QUERIES_HPP

namespace sparql_queries {

    string sparq_nodes(string fdrid) {
        string sparq = "# Find buses and phases\n"
            "PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
            "PREFIX c: <http://iec.ch/TC57/CIM100#>\n"
            "#SELECT ?fname ?cemrid ?cename ?busid ?busname ?phases WHERE {\n"
            "SELECT ?busid ?busname ?phases WHERE {\n"
            "  ?term c:Terminal.ConnectivityNode ?bus.\n"
            "  ?bus c:IdentifiedObject.name ?busname.\n"
            "  ?bus c:IdentifiedObject.mRID ?busid.\n"
            "  OPTIONAL { # Lines\n"
            "    ?cep c:ACLineSegmentPhase.ACLineSegment ?ce.\n"
            "    ?cep c:ACLineSegmentPhase.phase ?phsraw.\n"
            "    bind(strafter(str(?phsraw),\\\"SinglePhaseKind.\\\") as ?phases)\n"
            "  }\n"
            "  OPTIONAL { # Loads\n"
            "    ?cep c:EnergyConsumerPhase.EnergyConsumer ?ce.\n"
            "    ?cep c:EnergyConsumerPhase.phase ?phsraw.\n"
            "    bind(strafter(str(?phsraw),\\\"SinglePhaseKind.\\\") as ?phases)\n"
            "  }\n"
            "  OPTIONAL { # Switches\n"
            "    ?cep c:SwitchPhase.Switch ?ce.\n"
            "    ?cep c:SwitchPhase.phaseSide1 ?phs1raw.\n"
            "    bind(strafter(str(?phs1raw),\\\"SinglePhaseKind.\\\") as ?swPh1)\n"
            "    ?cep c:SwitchPhase.phaseSide1 ?phs2raw.\n"
            "    bind(strafter(str(?phs2raw),\\\"SinglePhaseKind.\\\") as ?swPh2)\n"
            "    bind(concat(str(?swPh1),str(?swPh2)) as ?phases)\n"
            "  }\n"
            "  OPTIONAL { # Capacitors\n"
            "    ?cep c:ShuntCompensatorPhase.ShuntCompensator ?ce.\n"
            "    ?cep c:ShuntCompensatorPhase.phase ?phsraw.\n"
            "    bind(strafter(str(?phsraw),\\\"SinglePhaseKind.\\\") as ?phases)\n"
            "  }\n"
            "  OPTIONAL { # Transformers and Tap Changers\n"
            "    ?tt c:TransformerTank.PowerTransformer ?ce.\n"
            "    ?tte c:TransformerTankEnd.TransformerTank ?tt.\n"
            "    ?tte c:TransformerTankEnd.phases ?phasesraw.\n"
            "    ?tte c:TransformerEnd.Terminal ?term\n"
            "    bind(strafter(str(?phasesraw),\\\"PhaseCode.\\\") as ?phases)\n"
            "  }\n"
            "  OPTIONAL { # Power electronics\n"
            "    ?cep c:PowerElectronicsConnectionPhase.PowerElectronicsConnection ?ce.\n"
            "    ?cep c:PowerElectronicsConnectionPhase.phase ?phsraw.\n"
            "    bind(strafter(str(?phsraw),\\\"SinglePhaseKind.\\\") as ?phases)\n"
            "  }\n"    
            "  VALUES ?fdrid {\\\""+fdrid+"\\\"}\n"
            "  ?term c:Terminal.ConductingEquipment ?ce.\n"
            "  ?ce c:Equipment.EquipmentContainer ?fdr.\n"
            "  ?ce c:IdentifiedObject.mRID ?cemrid.\n"
            "  ?ce c:IdentifiedObject.name ?cename.\n"
            "  ?fdr c:IdentifiedObject.mRID ?fdrid.\n"
            "  ?fdr c:IdentifiedObject.name ?fname\n"
            "}\n"
            "#GROUP BY ?fname ?cemrid ?cename ?busid ?busname ?phases\n"
            "GROUP BY ?busid ?busname ?phases\n"
            "#ORDER by ?fname ?busname\n"
            "ORDER by ?busname\n";
        return sparq;
    }


    string sparq_energy_consumer_pq(string fdrid) {
        string sparq =
        "PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
        "PREFIX c: <http://iec.ch/TC57/CIM100#>\n"
        "SELECT ?loadname ?busname ?conn ?phase ?p_3p ?q_3p ?p_phase ?q_phase WHERE {\n"
        "  # conducting equipment as an IdentifiedObject (PowerSystemResource)\n"
        "  ?econsumer c:IdentifiedObject.name ?loadname.\n"
        "  # terminals attached to conducting equipment\n"
        "  ?term c:Terminal.ConductingEquipment ?econsumer.\n"
        "  ?term c:Terminal.ConnectivityNode ?bus.\n"
        "  ?bus c:IdentifiedObject.name ?busname.\n"
        "  ?bus c:IdentifiedObject.mRID ?busid.\n"
        "  # p and q\n"
        "  ?econsumer c:EnergyConsumer.p ?p_3p.\n"
        "  ?econsumer c:EnergyConsumer.q ?q_3p.\n"
        "  # connection type\n"
        "  ?econsumer c:EnergyConsumer.phaseConnection ?connraw.\n"
        "  bind(strafter(str(?connraw),\\\"PhaseShuntConnectionKind.\\\") as ?conn)\n"
        "  # phases of the consumer\n"
        "  OPTIONAL {\n"
        "    ?ecp c:EnergyConsumerPhase.EnergyConsumer ?econsumer.\n"
        "    ?ecp c:EnergyConsumerPhase.phase ?phsraw.\n"
        "    bind(strafter(str(?phsraw),\\\"SinglePhaseKind.\\\") as ?phase)\n"
        "    ?ecp c:EnergyConsumerPhase.p ?p_phase.\n"
        "    ?ecp c:EnergyConsumerPhase.q ?q_phase.\n"
        "  }\n"
        "  VALUES ?fdrid {\\\""+fdrid+"\\\"} # 13 bus\n"
        "  ?econsumer c:Equipment.EquipmentContainer ?fdr.\n"
        "  ?fdr c:IdentifiedObject.mRID ?fdrid.\n"
        "}\n"
        "GROUP BY ?loadname ?busname ?conn ?phase ?p_3p ?q_3p ?p_phase ?q_phase\n"
        "ORDER by ?loadname\n";
        return sparq;
    }


    string sparq_ratio_tap_changer_nodes(string fdrid) { 
        string sparq = "# Find the nodes of each regulator\n"
            "PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
            "PREFIX c: <http://iec.ch/TC57/CIM100#>\n"
            "SELECT ?rtcname ?rtcid ?xtname ?cemrid ?primbus ?primphs ?regbus ?regphs WHERE {\n"
            "  # \n"
            "  ?rtc c:IdentifiedObject.name ?rtcname.\n"
            "  ?rtc c:IdentifiedObject.mRID ?rtcid.\n"
            "  ?rtc c:RatioTapChanger.TransformerEnd ?rte.\n"
            "  ?rte c:TransformerTankEnd.TransformerTank ?xt.\n"
            "  ?xt c:TransformerTank.PowerTransformer ?ce.\n"
            "  ?ce c:IdentifiedObject.mRID ?cemrid.\n"
            "  ?rte c:TransformerEnd.Terminal ?rterm.\n"
            "  ?rte c:TransformerTankEnd.phases ?rphsraw.\n"
            "  bind(strafter(str(?rphsraw),\\\"PhaseCode.\\\") as ?regphs)\n"
            "  ?rterm c:Terminal.ConnectivityNode ?rcn.\n"
            "  ?rcn c:IdentifiedObject.name ?regbus.\n"
            "  ?te c:TransformerTankEnd.TransformerTank ?xt.\n"
            "  ?te c:TransformerTankEnd.phases ?phsraw.\n"
            "  bind(strafter(str(?phsraw),\\\"PhaseCode.\\\") as ?primphs)\n"
            "  ?xt c:IdentifiedObject.name ?xtname.\n"
            "  ?te c:TransformerEnd.Terminal ?term.\n"
            "  ?term c:Terminal.ConnectivityNode ?cn.\n"
            "  ?cn c:IdentifiedObject.name ?primbus.\n"
            "  ?xt c:Equipment.EquipmentContainer ?fdr.\n"
            "  ?fdr c:IdentifiedObject.mRID ?fdrid.\n"
            "  VALUES ?fdrid {\\\""+fdrid+"\\\"}\n"
            "  FILTER ( ?primbus NOT IN ( ?regbus ) )\n"
            "}\n"
            "GROUP BY ?rtcname ?rtcid ?xtname ?cemrid ?primbus ?primphs ?regbus ?regphs\n"
            "ORDER by ?rtcname\n";
        return sparq;
    }


    string sparq_energy_source_buses(string fdrid) {
        string sparq =
            "# substation source - DistSubstation\n"
            "PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
            "PREFIX c:  <http://iec.ch/TC57/CIM100#>\n"
            "SELECT ?bus WHERE {\n"
            "?s r:type c:EnergySource.\n"
            "# feeder selection options - if all commented out, query matches all feeders\n"
            "  VALUES ?fdrid {\\\""+fdrid+"\\\"}  # 123 bus\n"
            "  ?s c:Equipment.EquipmentContainer ?fdr.\n"
            "  ?fdr c:IdentifiedObject.mRID ?fdrid.\n"
            "  ?t c:Terminal.ConductingEquipment ?s.\n"
            "  ?t c:Terminal.ConnectivityNode ?cn.\n"
            "  ?cn c:IdentifiedObject.name ?bus\n"
            "}\n"
            "ORDER by ?name\n";
        return sparq;
    }


    string sparq_cemrid_busnames(string fdrid) {
        string sparq =
            "PREFIX r:  <http://www.w3.org/1999/02/22-rdf-syntax-ns#> "
            "PREFIX c:  <http://iec.ch/TC57/CIM100#> "
            "SELECT ?cemrid ?busname "
            "WHERE { "
            "?term c:Terminal.ConnectivityNode ?bus. "
            "?bus c:IdentifiedObject.name ?busname. "
            "?bus c:IdentifiedObject.mRID ?busid. "
            "VALUES ?fdrid {\\\"" + fdrid + "\\\"} "
            "?term c:Terminal.ConductingEquipment ?ce. "
            "?ce c:Equipment.EquipmentContainer ?fdr. "
            "?fdr c:IdentifiedObject.mRID ?fdrid. "
            "?ce c:IdentifiedObject.mRID ?cemrid. "
            " } "
            "ORDER BY ?cemrid";
        return sparq;
    }
}
#endif
