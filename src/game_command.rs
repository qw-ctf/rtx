//! `gameExport_t` from `ktx/include/g_public.h` — the commands the engine sends to
//! `vmMain`. Order matches the C enum exactly (0..=20).

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GameCommand {
    /// `( int levelTime, int randomSeed, int restart )` — returns `gameData_t*`.
    Init,
    /// Parse the map entity string and spawn entities.
    LoadEntities,
    Shutdown,
    /// `( int isSpectator )`
    ClientConnect,
    PutClientInServer,
    ClientUserInfoChanged,
    ClientDisconnect,
    ClientCommand,
    ClientPreThink,
    ClientThink,
    ClientPostThink,
    /// `( int levelTime, int isBotFrame )`
    StartFrame,
    SetChangeParams,
    SetNewParams,
    /// Unrecognized console command; return false if not handled.
    ConsoleCommand,
    /// `(self, other)`
    EdictTouch,
    /// `(self, other = world, time)`
    EdictThink,
    /// `(self, other)`
    EdictBlocked,
    /// `( int isTeamSay )`
    ClientSay,
    /// `( int duration_msec )`
    PausedTic,
    /// `(self)`
    ClearEdict,
}

impl GameCommand {
    /// Map the raw engine command id to a known variant. Unknown ids (e.g.
    /// `GAME_EDICT_CSQCSEND = 200`) yield `None` so the dispatcher can ignore them
    /// safely instead of transmuting an out-of-range discriminant.
    pub fn from_i32(v: i32) -> Option<GameCommand> {
        use GameCommand::*;
        Some(match v {
            0 => Init,
            1 => LoadEntities,
            2 => Shutdown,
            3 => ClientConnect,
            4 => PutClientInServer,
            5 => ClientUserInfoChanged,
            6 => ClientDisconnect,
            7 => ClientCommand,
            8 => ClientPreThink,
            9 => ClientThink,
            10 => ClientPostThink,
            11 => StartFrame,
            12 => SetChangeParams,
            13 => SetNewParams,
            14 => ConsoleCommand,
            15 => EdictTouch,
            16 => EdictThink,
            17 => EdictBlocked,
            18 => ClientSay,
            19 => PausedTic,
            20 => ClearEdict,
            _ => return None,
        })
    }
}
