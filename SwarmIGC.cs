// Inter-grid communication helper for DragonSwarm
// Defines message format and helper send/receive routines

static class SwarmIGC
{
    public const string CHANNEL = "DRAGONSWARM"; // shared IGC channel

    public struct Message
    {
        public string Command; // e.g. HELLO, ASSIGN, HOST, FIRE
        public long Sender;    // entityId of sender
        public string Data;    // optional payload
    }

    // Compose and send a message using string packing
    public static void Send(IMyIntergridCommunicationSystem igc, Message msg)
    {
        string payload = msg.Sender.ToString() + "|" + msg.Command + "|" + (msg.Data ?? string.Empty);
        igc.SendBroadcastMessage(CHANNEL, payload, TransmissionDistance.TransmissionDistanceMax);
    }

    // Try to parse an incoming IGC message
    public static bool TryParse(MyIGCMessage src, ref Message msg)
    {
        if (src.Tag != CHANNEL || src.Data == null) return false;
        string text = src.Data as string;
        if (text == null) return false;
        string[] parts = text.Split('|');
        if (parts.Length < 2) return false;
        long sender;
        if (!long.TryParse(parts[0], out sender)) return false;
        msg.Sender = sender;
        msg.Command = parts[1];
        msg.Data = parts.Length > 2 ? parts[2] : string.Empty;
        return true;
    }
}
