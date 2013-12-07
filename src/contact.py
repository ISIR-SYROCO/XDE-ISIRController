#!/xde

""" Module to update the contact information vectors and matrices.

It should generally be used with a ContactAvoidanceConstraint, to update
the constraint matrices.
"""

import xdefw.rtt
import lgsm

from xde_world_manager.collision import alignz


class ContactAvoidanceConstraintUpdater:

    def __init__(self, CAConstraint, model, physic_agent, connector_name="ContactAvoidanceConnector", port_name="ContactAvoidancePort"):
        self.CAConstraint      = CAConstraint
        self.model             = model
        self.ndof              = self.model.nbDofs()
        self.seg_names         = [self.model.getSegmentName(i) for i in range(self.model.nbSegments())]
        self.phy               = physic_agent
        self.contact_connector = self.phy.s.Connectors.OConnectorContactBody.new(connector_name, port_name+".OutputPort")
        self.contact_port      = xdefw.rtt.InputPort.new(port_name+".InputPort", "SMsg")
        self.contact_port.connectTo(self.phy.getPort(port_name+".OutputPort"))


    def _get_data_from_cpt_(self, entity, p_0_segcpt, n_0_segcpt):
        if entity in self.seg_names:
            idx                 = self.model.getSegmentIndex(entity)
            H_0_seg             = self.model.getSegmentPosition(idx)
            T_seg_0_seg         = self.model.getSegmentVelocity(idx)
            J_seg_0_seg         = self.model.getSegmentJacobian(idx)
            JdotQdot_seg_0_seg  = self.model.getSegmentJdotQdot(idx)

            H_0_segcpt = lgsm.Displacement()
            H_0_segcpt.setTranslation(p_0_segcpt)
            H_0_segcpt.setRotation(alignz(n_0_segcpt))

            H_seg_segcpt  = H_0_seg.inverse() * H_0_segcpt
            Ad_segcpt_seg = H_seg_segcpt.inverse().adjoint()

            J_segcpt_0_segcpt         = Ad_segcpt_seg * J_seg_0_seg
            JdotQdot_segcpt_0_segcpt  = Ad_segcpt_seg * JdotQdot_seg_0_seg
            T_segcpt_0_segcpt         = Ad_segcpt_seg * T_seg_0_seg
            
            Jcpt     = J_segcpt_0_segcpt[5,:]
            dJdqcpt  = JdotQdot_segcpt_0_segcpt.vz  #dJdqcpt[5,:]
            velcpt   = T_segcpt_0_segcpt.vz         #T_segcpt_0_segcpt[5] #or .vz
        else:
            Jcpt    = lgsm.zeros(self.ndof).T
            dJdqcpt = 0.
            velcpt  = 0.

        return Jcpt, dJdqcpt, velcpt


    def update(self):
        #WARNING: maybe an other way with 'cpd = self._wm.phy.s.XCD.CompositePairDescriptor(cpd_str)' --> see WorldManager.collision
#        print dir(self.contact_port)
        smsg, smsg_ok = self.contact_port.read() # check for synchronization???
        # we suppose it is always ok...
        
        N      = len(smsg.cpt)
        J      = lgsm.np.matrix(lgsm.np.zeros((N, self.ndof)))
        dJdq   = lgsm.zeros(N)
        CAdist = lgsm.zeros(N)
        CAvel  = lgsm.zeros(N)

        for l, cpt in enumerate(smsg.cpt):
            Ji, dJdqi, CAveli = self._get_data_from_cpt_(str(cpt.entity_i), lgsm.vector(*cpt.ai), lgsm.vector(*cpt.ni))
            Jj, dJdqj, CAvelj = self._get_data_from_cpt_(str(cpt.entity_j), lgsm.vector(*cpt.aj), lgsm.vector(*cpt.nj))

            J[l, :]   = Ji     + Jj
            dJdq[l]   = dJdqi  + dJdqj
            CAdist[l] = cpt.gap
            CAvel[l]  = CAveli + CAvelj


        self.CAConstraint.updateContactInformation(J, dJdq, CAdist, CAvel)


    def add_contactAvoidance(self, body1, body2):
        self.contact_connector.addInteraction(body1, body2)

    def remove_contactAvoidance(self, body1, body2):
        self.contact_connector.removeInteraction(vbody1, body2)






