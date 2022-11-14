class A2_LINK:
    def __init__(self, a2_link):
        for key in a2_link.keys():
            if key[:2] == 'R_':
                R_LinkID = key
            
            elif key[:2] == 'L_':
                L_LinkID = key

        self.ID = a2_link['ID']
        self.AdminCode = a2_link['AdminCode']
        self.RoadRank = a2_link['RoadRank']
        self.RoadType = a2_link['RoadType']
        self.RoadNo = a2_link['RoadNo']
        self.LinkType = a2_link['LinkType']
        self.MaxSpeed = a2_link['MaxSpeed']
        self.LaneNo = a2_link['LaneNo']
        self.R_LinkID = a2_link[R_LinkID]
        self.L_LinkID = a2_link[L_LinkID]
        self.FromNodeID = a2_link['FromNodeID']
        self.ToNodeID = a2_link['ToNodeID']
        self.SectionID = a2_link['SectionID']
        self.Length = a2_link['Length']
        self.ITSLinkID = a2_link['ITSLinkID']
        self.Maker = a2_link['Maker']
        self.UpdateDate = a2_link['UpdateDate']
        self.Version = a2_link['Version']
        self.Remark = a2_link['Remark']
        self.geometry = a2_link['geometry']