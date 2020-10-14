use serde::{Serialize, Deserialize};

#[derive(Serialize, Deserialize, Debug)]
pub enum RemoteCommand {
	Arm(bool),
	ReturnToHome,
}