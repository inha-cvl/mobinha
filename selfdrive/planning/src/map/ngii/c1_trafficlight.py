class C1_TRAFFICLIGHT:
    def __init__(self, c1_trafficlight):
        # error mapping
        PostID = 'PostID'
        if PostID not in c1_trafficlight.keys():
            PostID = 'postID'

        self.ID = c1_trafficlight['ID']
        self.AdminCode = c1_trafficlight['AdminCode']
        self.Type = c1_trafficlight['Type']
        self.LinkID = c1_trafficlight['LinkID']
        self.Ref_Lane = c1_trafficlight['Ref_Lane']
        self.PostID = c1_trafficlight[PostID]
        self.Maker = c1_trafficlight['Maker']
        self.UpdateDate = c1_trafficlight['UpdateDate']
        self.Version = c1_trafficlight['Version']
        self.Remark = c1_trafficlight['Remark']
        self.geometry = c1_trafficlight['geometry']